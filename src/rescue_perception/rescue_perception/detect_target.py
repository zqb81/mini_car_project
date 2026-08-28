#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""救援目标检测：YOLO 检测 + 深度投影，输出 map 系目标位姿。

业务目的：
  救援场景不知道目标（人/物体）在哪里，因此不能依赖需要人工初始化的跟踪器
  （KCF 必须先有人框选目标才能跟）。本节点让相机自主发现目标，并把检测结果
  转换成导航可以直接消费的 map 系位姿，形成「检测 -> 规划」链路：

    /camera/color/image_raw ┐
    /camera/depth/image_raw ├─> YOLO 检测 -> 深度反投影 -> TF -> map 系位姿
    /camera/color/camera_info ┘                                    |
                                                                   v
                                       /rescue/target_pose（供 Nav2 / FollowTarget）

  本节点只做感知，不发布任何速度指令——运动始终由 Nav2 独占，避免出现第二个
  控制器争抢 /cmd_vel。

坐标链路与前提：
  1. 反投影用彩色相机内参，得到的三维点位于**彩色光学坐标系**；
  2. 深度图必须已对齐到彩色视角（astra_pro.launch.py 的 depth_align=true），
     否则彩色像素与深度像素不对应，测距会偏。本节点对尺寸不一致做了等比
     缩放兜底，但那只是粗略可用，正确做法仍是开启对齐。
  3. 由 TF 变换到 target_frame（默认 map），依赖 map->odom->base_footprint->
     camera 链路完整。

算力说明：
  树莓派 CPU 上 YOLOv8n 约个位数 FPS，因此默认限流到 2Hz（min_interval）。
  需要更高帧率请降低分辨率、改用 Coral/Hailo 加速，或换用自带 VPU 的相机。
"""

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped, PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

# 深度消息编码 -> 缩放系数（转成米）。Astra 默认输出 16UC1，单位为毫米。
_DEPTH_SCALE = {"16UC1": 0.001, "32FC1": 1.0}


class DetectTargetNode(Node):
    """YOLO 检测 + 深度投影节点。"""

    def __init__(self):
        super().__init__("detect_target")

        # ---- 话题与模型 ----
        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        # 默认 yolov8n：nano 版，CPU 上最轻，救援检测精度与速度的最佳折中
        self.declare_parameter("model", "yolov8n.pt")

        # ---- 检测过滤 ----
        # 默认只找人。COCO 类别名，会按模型自带的 names 表反查索引。
        # 用逗号分隔的字符串而非列表：列表类型从 launch/命令行传参容易
        # 与参数声明的类型推断冲突，字符串两种场景都能用。
        self.declare_parameter("target_classes", "person")
        self.declare_parameter("conf_threshold", 0.5)
        # 检测最小间隔（秒）：限流，避免 CPU 被 YOLO 占满
        self.declare_parameter("min_interval", 0.5)

        # ---- 深度与坐标 ----
        self.declare_parameter("min_depth", 0.3)      # 米，过近的深度不可靠
        self.declare_parameter("max_depth", 8.0)      # 米，Astra 有效量程内
        self.declare_parameter("depth_patch_radius", 4)  # 取中位数的邻域半径（像素）
        self.declare_parameter("target_frame", "map")

        # ---- 输出 ----
        self.declare_parameter("publish_debug_image", True)

        self._conf = self.get_parameter("conf_threshold").value
        self._min_interval = self.get_parameter("min_interval").value
        self._min_depth = self.get_parameter("min_depth").value
        self._max_depth = self.get_parameter("max_depth").value
        self._patch_r = int(self.get_parameter("depth_patch_radius").value)
        self._target_frame = self.get_parameter("target_frame").value
        self._publish_debug = self.get_parameter("publish_debug_image").value

        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ---- 加载模型（放在订阅之前，失败即快速退出）----
        from ultralytics import YOLO

        model_path = self.get_parameter("model").value
        try:
            self._model = YOLO(model_path)
        except Exception as exc:  # 模型文件缺失或 ultralytics 未安装
            raise RuntimeError(
                f"无法加载 YOLO 模型 '{model_path}'：{exc}。"
                f"请确认已 pip install ultralytics 且模型文件存在。"
            ) from exc

        # 类别名 -> COCO 索引；无效的类别名直接告警，避免静默什么都不检测
        raw_classes = self.get_parameter("target_classes").value
        self._class_ids = self._resolve_class_ids(self._split_classes(raw_classes))
        self.get_logger().info(
            f"目标类别 {raw_classes!r} -> 索引 {self._class_ids}"
        )

        # ---- 同步订阅：RGB + 深度 + 内参 ----
        rgb_topic = self.get_parameter("rgb_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        self._sync = ApproximateTimeSynchronizer(
            [
                Subscriber(self, Image, rgb_topic),
                Subscriber(self, Image, depth_topic),
                Subscriber(self, CameraInfo, info_topic),
            ],
            queue_size=10,
            slop=0.1,  # RGB 与深度时间戳常有偏差，用近似同步
        )
        self._sync.registerCallback(self._on_frames)

        # ---- 发布 ----
        self._pub_detections = self.create_publisher(
            Detection3DArray, "~/detections_3d", 10
        )
        # 目标位姿用全局名：它是给导航层消费的公共契约，不是节点私有话题
        self._pub_target = self.create_publisher(
            PoseStamped, "/rescue/target_pose", 10
        )
        self._pub_debug = self.create_publisher(Image, "~/debug_image", 10)

        self._last_detect_time = 0.0
        self._depth_unit_warned = False
        # 彩色图分辨率，供深度图坐标等比映射使用（每帧更新）
        self._rgb_width = 0
        self._rgb_height = 0

    # ---- 参数处理 ------------------------------------------------------

    @staticmethod
    def _split_classes(raw):
        """解析类别参数：命令行传字符串，参数文件/YAML 传列表，两者都支持。"""
        if isinstance(raw, str):
            return [n.strip() for n in raw.split(",") if n.strip()]
        return [str(n) for n in (raw or [])]

    def _resolve_class_ids(self, names):
        """把类别名（如 person）映射为模型自带的类别索引。"""
        name_to_id = {v: k for k, v in self._model.names.items()}
        ids = []
        for n in names:
            if n in name_to_id:
                ids.append(name_to_id[n])
            else:
                self.get_logger().warning(
                    f"类别 '{n}' 不在模型类别表中，已忽略。"
                    f"可用类别示例：{list(self._model.names.values())[:10]}"
                )
        return ids

    # ---- 主回调 --------------------------------------------------------

    def _on_frames(self, rgb_msg, depth_msg, info_msg):
        """同步到一帧 RGB+深度+内参后做一次检测（按 min_interval 限流）。"""
        now = time.monotonic()
        if now - self._last_detect_time < self._min_interval:
            return
        self._last_detect_time = now

        try:
            rgb = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth = self._bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough"
            )
        except CvBridgeError as exc:
            self.get_logger().warning(f"图像转换失败：{exc}")
            return

        depth_m = self._to_meters(depth, depth_msg.encoding)
        if depth_m is None:
            return

        # 记录彩色图分辨率：深度图分辨率可能不同，采样时需按比例映射
        self._rgb_height, self._rgb_width = rgb.shape[:2]

        # 反投影用彩色内参：K = [fx 0 cx; 0 fy cy; 0 0 1]
        k = info_msg.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        if fx == 0.0 or fy == 0.0:
            self.get_logger().warning("相机内参为空，跳过本帧。")
            return

        results = self._model.predict(
            rgb, conf=self._conf, classes=self._class_ids or None, verbose=False
        )

        detections = Detection3DArray()
        detections.header.stamp = rgb_msg.header.stamp
        detections.header.frame_id = self._target_frame

        debug_img = rgb.copy() if self._publish_debug else None
        best_target = None  # 置信度最高的目标，用于发布单一目标位姿

        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue
            for box in boxes:
                det3d, pose_map, z = self._build_detection(
                    box, rgb_msg, depth_m, (fx, fy, cx, cy)
                )
                if det3d is None:
                    continue
                detections.detections.append(det3d)

                score = float(box.conf[0])
                if best_target is None or score > best_target[0]:
                    best_target = (score, pose_map)

                if debug_img is not None:
                    self._draw(debug_img, box, z, score)

        if detections.detections:
            self._pub_detections.publish(detections)
            if best_target is not None:
                self._pub_target.publish(best_target[1])

        if debug_img is not None:
            try:
                out = self._bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
                out.header = rgb_msg.header
                self._pub_debug.publish(out)
            except CvBridgeError as exc:
                self.get_logger().warning(f"调试图像发布失败：{exc}")

    # ---- 深度与投影 ----------------------------------------------------

    def _to_meters(self, depth, encoding):
        """深度图转米。未知编码时告警并跳过，避免把毫米当米用导致测距错 1000 倍。"""
        scale = _DEPTH_SCALE.get(encoding)
        if scale is None:
            if not self._depth_unit_warned:
                self.get_logger().warning(
                    f"未知深度编码 '{encoding}'，无法确定单位，跳过深度处理。"
                    f"支持：{list(_DEPTH_SCALE)}"
                )
                self._depth_unit_warned = True
            return None
        return depth.astype(np.float32) * scale

    def _build_detection(self, box, rgb_msg, depth_m, intrinsics):
        """把单个检测框转换为 Detection3D 与 map 系位姿。

        返回 (det3d, pose_map, 深度值)；深度无效时返回 (None, None, None)。
        """
        fx, fy, cx, cy = intrinsics
        x1, y1, x2, y2 = box.xyxy[0].tolist()

        # 取框中心附近邻域的中位数：单点深度易受空洞和噪声影响
        cu, cv = int((x1 + x2) / 2.0), int((y1 + y2) / 2.0)
        z = self._sample_depth(depth_m, cu, cv)
        if z is None:
            return None, None, None

        # 反投影到彩色光学坐标系（X 右、Y 下、Z 前）
        X = (cu - cx) * z / fx
        Y = (cv - cy) * z / fy

        source_frame = rgb_msg.header.frame_id
        stamp = rgb_msg.header.stamp
        try:
            tf = self._tf_buffer.lookup_transform(
                self._target_frame, source_frame, stamp
            )
        except Exception as exc:
            self.get_logger().warning(
                f"TF 变换失败（{source_frame} -> {self._target_frame}）：{exc}"
            )
            return None, None, None

        point = PointStamped()
        point.header.frame_id = source_frame
        point.header.stamp = stamp
        point.point.x = float(X)
        point.point.y = float(Y)
        point.point.z = float(z)
        try:
            point_map = do_transform_point(point, tf)
        except Exception as exc:
            self.get_logger().warning(f"点变换失败：{exc}")
            return None, None, None

        score = float(box.conf[0])
        cls_id = int(box.cls[0])
        cls_name = self._model.names.get(cls_id, str(cls_id))

        pose = PoseStamped()
        pose.header.frame_id = self._target_frame
        pose.header.stamp = stamp
        pose.pose.position.x = point_map.point.x
        pose.pose.position.y = point_map.point.y
        pose.pose.position.z = point_map.point.z
        # 目标朝向未知，给一个朝上的单位四元数，避免下游拿到未初始化的姿态
        pose.pose.orientation.w = 1.0

        det = Detection3D()
        det.header.frame_id = self._target_frame
        det.header.stamp = stamp
        hypo = ObjectHypothesisWithPose()
        hypo.hypothesis.class_id = cls_name
        hypo.hypothesis.score = score
        hypo.pose.pose.position.x = point_map.point.x
        hypo.pose.pose.position.y = point_map.point.y
        hypo.pose.pose.position.z = point_map.point.z
        hypo.pose.pose.orientation.w = 1.0
        det.results.append(hypo)
        # 3D 框尺寸用深度估算：横向尺寸由像素宽按相似三角形换算
        bbox = BoundingBox3D()
        width_px = x2 - x1
        height_px = y2 - y1
        bbox.center.position.x = point_map.point.x
        bbox.center.position.y = point_map.point.y
        bbox.center.position.z = point_map.point.z
        bbox.center.orientation.w = 1.0
        bbox.size.x = float(width_px * z / fx)
        bbox.size.y = float(height_px * z / fy)
        bbox.size.z = 0.5  # 深度方向尺寸无法由单视角恢复，给一个占位值
        det.bbox = bbox

        return det, pose, z

    def _sample_depth(self, depth_m, cu, cv):
        """在彩色像素 (cu,cv) 对应的深度位置取邻域中位数。"""
        h, w = depth_m.shape[:2]
        # 深度图与彩色图分辨率可能不同，等比映射到深度图坐标后再采样
        u = int(round(cu * w / max(self._rgb_width, 1)))
        v = int(round(cv * h / max(self._rgb_height, 1)))
        r = self._patch_r
        patch = depth_m[max(0, v - r):v + r + 1, max(0, u - r):u + r + 1]
        if patch.size == 0:
            return None
        valid = patch[(patch > self._min_depth) & (patch < self._max_depth)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _draw(self, img, box, z, score):
        """在调试图上画框与标签，便于 RViz/rqt_image_view 直观检查。"""
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        cls_id = int(box.cls[0])
        name = self._model.names.get(cls_id, str(cls_id))
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)),
                      (0, 255, 0), 2)
        cv2.putText(
            img, f"{name} {score:.2f} {z:.2f}m",
            (int(x1), max(0, int(y1) - 6)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA,
        )


def main(args=None):
    rclpy.init(args=args)
    node = DetectTargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
