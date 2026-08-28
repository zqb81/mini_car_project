#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检测与导航的融合：把检测结果按置信度分级，自动衔接两阶段融合跟随。

业务目的：
  detect_target 只回答「画面里有什么、在哪」，不决定要不要过去。本节点补上
  决策层，形成完整的「检测 -> 决策 -> 规划」链路：

    /detect_target/detections_3d ─> 置信度分级 ─> 稳定性校验 ─> FollowTarget
                                        │                          (Nav2 导航 + 视觉伺服)
                                        └─> /rescue/pending_target（待人工确认）

为什么需要置信度分级：
  救援场景误检代价很高——机器人冲向一个误检的「人」可能错过真正的伤员，
  甚至在狭窄空间造成碰撞。因此不能检测到就冲过去，必须分级处理。

三级策略（阈值可配）：
  score >= auto_conf      连续稳定后出现即自动下发导航目标（rescue 效率优先）
  score >= confirm_conf   记为待确认目标，等人工确认后再行动（默认区间）
  score <  confirm_conf   直接丢弃

为什么需要稳定性校验：
  单帧检测可能是闪烁误检。要求连续 N 次检测位置相近（stability_radius 内）
  才认定为稳定目标，可显著抑制偶发误检。

安全边界：
  本节点不发布速度指令，只下发 FollowTarget action；运动始终由 Nav2（staging
  阶段）与视觉伺服环（servo 阶段）按阶段接管。
"""

import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool
from vision_msgs.msg import Detection3DArray

from kcf_track.action import FollowTarget


class TargetFusionNode(Node):
    """检测结果到导航目标 decision 层。"""

    def __init__(self):
        super().__init__("target_fusion")

        # ---- 输入/阈值 ----
        self.declare_parameter(
            "detections_topic", "/detect_target/detections_3d"
        )
        self.declare_parameter("auto_conf_threshold", 0.75)      # 自动行动
        self.declare_parameter("confirm_conf_threshold", 0.40)   # 需人工确认
        self.declare_parameter("min_stable_count", 3)            # 连续稳定帧数
        self.declare_parameter("stability_radius", 0.5)          # 位置稳定判定半径（米）
        self.declare_parameter("auto_mode", True)                # 是否允许自动行动

        # ---- 下发给 FollowTarget 的参数 ----
        self.declare_parameter("follow_distance", 1.2)           # 期望保持距离
        self.declare_parameter("servo_timeout", 60.0)            # 伺服阶段超时
        self.declare_parameter("nav2_server_timeout", 5.0)

        self._auto_conf = self.get_parameter("auto_conf_threshold").value
        self._confirm_conf = self.get_parameter("confirm_conf_threshold").value
        self._min_stable = int(self.get_parameter("min_stable_count").value)
        self._radius = self.get_parameter("stability_radius").value
        self._auto_mode = self.get_parameter("auto_mode").value
        self._follow_distance = self.get_parameter("follow_distance").value
        self._servo_timeout = self.get_parameter("servo_timeout").value

        self._group = ReentrantCallbackGroup()
        self._sub_det = self.create_subscription(
            Detection3DArray,
            self.get_parameter("detections_topic").value,
            self._on_detections,
            10,
            callback_group=self._group,
        )
        # 人工确认入口：置 true 即对当前待确认目标下发导航
        self._sub_confirm = self.create_subscription(
            Bool, "/rescue/confirm", self._on_confirm, 10,
            callback_group=self._group,
        )

        self._pub_pending = self.create_publisher(
            PoseStamped, "/rescue/pending_target", 10
        )

        self._nav_client = ActionClient(
            self, FollowTarget, "follow_target", callback_group=self._group
        )

        # ---- 状态 ----
        self._stable_pos = None      # 当前稳定目标的累计位置
        self._stable_count = 0
        self._pending = None         # 待人工确认的目标（PoseStamped）
        self._goal_active = False    # 是否已有 FollowTarget 在执行
        self._last_sent_time = 0.0
        self._cooldown = 10.0        # 同一目标下发后的冷却，避免重复触发

        self.get_logger().info(
            f"融合节点就绪：自动阈值 {self._auto_conf}，"
            f"确认阈值 {self._confirm_conf}，需连续稳定 {self._min_stable} 次"
        )

    # ---- 检测回调 ------------------------------------------------------

    def _on_detections(self, msg):
        """取置信度最高的检测结果，按分级策略处理。"""
        best = None
        for det in msg.detections:
            if not det.results:
                continue
            score = float(det.results[0].hypothesis.score)
            if best is None or score > best[0]:
                best = (score, det)

        if best is None:
            return

        score, det = best
        if score < self._confirm_conf:
            return  # 低置信度直接丢弃

        pos = det.results[0].pose.pose.position
        pose = self._make_pose(msg, pos)

        if self._update_stability(pos) is False:
            return  # 尚未稳定，继续累积

        # 已稳定：按置信度分级
        if score >= self._auto_conf and self._auto_mode:
            self._send_follow(pose, reason=f"自动（score={score:.2f}）")
        else:
            self._pending = pose
            self._pub_pending.publish(pose)
            # 区分两种原因，避免日志误导排障：置信度不足 vs 自动模式关闭
            reason = (
                "自动模式已关闭（auto_mode=false）"
                if not self._auto_mode
                else f"置信度 {score:.2f} 低于自动阈值 {self._auto_conf}"
            )
            self.get_logger().info(
                f"目标待确认（{reason}），"
                f"发布 /rescue/pending_target 等待人工确认"
            )

    def _make_pose(self, msg, pos):
        """构造 map 系目标位姿。朝向未知，给单位四元数。"""
        pose = PoseStamped()
        pose.header.frame_id = msg.header.frame_id
        pose.header.stamp = msg.header.stamp
        pose.pose.position.x = pos.x
        pose.pose.position.y = pos.y
        pose.pose.position.z = pos.z
        pose.pose.orientation.w = 1.0
        return pose

    def _update_stability(self, pos):
        """累计连续稳定帧数。返回 True 表示已达稳定阈值。"""
        if self._stable_pos is None:
            self._stable_pos = (pos.x, pos.y, pos.z)
            self._stable_count = 1
            return False

        dx = pos.x - self._stable_pos[0]
        dy = pos.y - self._stable_pos[1]
        dz = pos.z - self._stable_pos[2]
        if (dx * dx + dy * dy + dz * dz) ** 0.5 <= self._radius:
            self._stable_count += 1
        else:
            # 位置跳变：视为新目标，重新开始累计
            self._stable_pos = (pos.x, pos.y, pos.z)
            self._stable_count = 1
            return False

        if self._stable_count >= self._min_stable:
            return True
        self.get_logger().debug(
            f"目标稳定中 {self._stable_count}/{self._min_stable}"
        )
        return False

    # ---- 确认与下发 ----------------------------------------------------

    def _on_confirm(self, msg):
        """人工确认：对当前待确认目标下发导航。"""
        if not msg.data:
            return
        if self._pending is None:
            self.get_logger().warning("收到确认但没有待确认目标，已忽略。")
            return
        self._send_follow(self._pending, reason="人工确认")
        self._pending = None

    def _send_follow(self, pose, reason):
        """下发 FollowTarget 目标，带冷却与去重。"""
        now = time.monotonic()
        if self._goal_active:
            return
        if now - self._last_sent_time < self._cooldown:
            return

        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning("FollowTarget 动作服务不可用。")
            return

        goal = FollowTarget.Goal()
        goal.use_staging_pose = True
        goal.staging_pose = pose
        goal.target_distance = float(self._follow_distance)
        goal.servo_timeout = float(self._servo_timeout)

        self._last_sent_time = now
        self._goal_active = True
        self.get_logger().info(
            f"下发跟随目标（{reason}）："
            f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
        )
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self._goal_active = False
            self.get_logger().warning("FollowTarget 目标被拒绝。")
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        self._goal_active = False
        # 目标丢失/超时/取消后清空稳定累计，重新观察
        self._stable_pos = None
        self._stable_count = 0
        try:
            result = future.result().result
            self.get_logger().info(
                f"跟随结束：success={result.success} "
                f"error_code={result.error_code}"
            )
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TargetFusionNode()
    # 用多线程执行器：action 回调与订阅回调可并发，避免等待结果时
    # 阻塞检测回调导致目标更新停滞。
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
