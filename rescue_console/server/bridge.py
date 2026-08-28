# -*- coding: utf-8 -*-
"""桥接层：网关与 ROS2 真实小车之间的数据通路。

业务目的：
  救援控制台网关通过 rclpy 接入目标机的 ROS2 话题与动作，使浏览器客户端
  无需任何 ROS 依赖即可遥测、遥控与下发导航目标。本文件定义桥接接口
  BaseBridge，并提供面向目标机的唯一实现 RosBridge：
    - RosBridge：目标机（Ubuntu 22.04 + ROS2 Humble，aarch64）用，通过
      rclpy 订阅 /odom、/PowerVoltage、/chassis_enabled、/scan、/map，
      发布 /cmd_vel，导航目标走 Nav2 NavigateToPose action。

  运行前提：必须在 source 过 ROS2 Humble 与 colcon 工作空间的环境里启动，
  否则 rclpy 导入失败并抛出中文提示（见 RosBridge.__init__）。

接口约定（见同目录 app.py）：
  tick(dt)         由服务端以固定周期调用，推进内部状态
  telemetry()      返回遥测字典（位姿、速度、电量、使能、激光）
  map_message()    返回占用栅格地图消息；无更新时返回 None
  cmd_vel(vx,vy,wz) 手动遥控指令（米/秒、弧度/秒）
  nav_goal(x,y)    下发导航目标点（地图坐标，米）
  cancel_nav()     取消导航
  close()          服务退出时释放资源（rclpy 等），默认空实现
"""

from __future__ import annotations

import math
import time
from typing import Optional


# ---------------------------------------------------------------------------
# 全局常量
# ---------------------------------------------------------------------------
_CMD_VEL_TIMEOUT = 0.5    # 手动指令超时（秒）：超时未收到新指令则自动停车
_BATTERY_MIN = 10.0       # 低电量阈值（伏），仅用于遥测中的 battery_low 提示


class BaseBridge:
    """桥接接口。所有方法必须线程安全或仅在事件循环内调用。"""

    name = "base"

    def tick(self, dt: float) -> None:  # pragma: no cover - 接口定义
        raise NotImplementedError

    def telemetry(self) -> dict:  # pragma: no cover - 接口定义
        raise NotImplementedError

    def map_message(self, force: bool = False) -> Optional[dict]:
        """占用栅格地图消息；无更新且未强制时返回 None。"""
        raise NotImplementedError  # pragma: no cover - 接口定义

    def cmd_vel(self, vx: float, vy: float, wz: float) -> None:  # pragma: no cover
        raise NotImplementedError

    def nav_goal(self, x: float, y: float) -> None:  # pragma: no cover
        raise NotImplementedError

    def cancel_nav(self) -> None:  # pragma: no cover
        raise NotImplementedError

    def close(self) -> None:
        """释放资源；无外部资源的实现无需覆盖。"""


class RosBridge(BaseBridge):
    """ROS2 桥接：目标机（树莓派 + Ubuntu 22.04 + ROS2 Humble）使用。

    数据通路（与 docs/AGENT_HANDOFF.md 第 7 节话题契约一致）：
      订阅 /odom /PowerVoltage /chassis_enabled /scan /map /tf
      发布 /cmd_vel（geometry_msgs/msg/Twist）
      导航目标走 Nav2 bt_navigator 的 NavigateToPose action，
      支持结果回调（到达/失败/取消）与取消。

    线程模型：
      rclpy.spin 在独立守护线程运行，订阅回调只更新缓存；
      FastAPI 事件循环只读缓存或调用 action/publisher，线程安全。

    位姿来源：优先查 TF map -> base_footprint（含 RTAB-Map 修正），
    无 TF 时回退 /odom 原始位姿并标注 pose_frame="odom"。

    激光安装角偏移：本项目 A1M8 安装 yaw=pi（README 16.2 随车默认值，
    待实测复核），激光系角度加偏移后才等于机器人系角度，可通过环境
    变量 RESCUE_LASER_YAW_OFFSET（弧度）覆盖。

    已知限制（部署说明中同步标注）：
      - 本网关、Nav2、KCF 都可能发布 /cmd_vel，无 twist_mux 仲裁，
        手动遥控与自动导航不得同时进行（手动输入会先取消 Nav2 目标）。
      - /map 的 RLE 编码在 Python 中逐格进行，RTAB-Map 大地图（>1M 格）
        单帧编码耗时约 0.1~0.3 秒，会短暂阻塞事件循环，属可接受范围。
    """

    name = "ros"

    def __init__(self) -> None:
        import math
        import os
        import threading

        try:
            import rclpy
        except ImportError as exc:
            raise RuntimeError(
                "ROS2 桥接需要 rclpy（目标机 Ubuntu 22.04 + ROS2 Humble，"
                "需先 source /opt/ros/humble/setup.bash 与 colcon 工作空间"
                "的 install/setup.bash，再启动本服务）。"
            ) from exc

        from geometry_msgs.msg import Twist
        from nav_msgs.msg import OccupancyGrid, Odometry
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import Bool, Float32
        from tf2_ros import Buffer, TransformListener

        self._rclpy = rclpy
        self._Twist = Twist
        self._NavigateToPose = NavigateToPose
        self._laser_offset = float(
            os.environ.get("RESCUE_LASER_YAW_OFFSET", math.pi)
        )
        self._closed = False

        # ---- 缓存（订阅回调线程写，事件循环读）----
        self._pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self._pose_frame = "odom"
        self._twist = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        self._battery: Optional[float] = None
        self._enabled = True
        self._scan = None
        self._map_flat = None
        self._map_meta = None
        self._map_dirty = False
        self._map_cache: Optional[dict] = None
        self._manual = False
        self._last_cmd = 0.0
        self._goal = None
        self._goal_handle = None
        self._nav_state = "idle"
        self._odo = 0.0
        self._odo_prev = None

        rclpy.init()
        node = rclpy.create_node("rescue_gateway")
        self._node = node

        # /map 发布方可能 latched（transient_local），订阅端用不低于发布端
        # 持久性的 QoS 才能同时匹配 volatile 与 transient_local 发布者
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(Odometry, "/odom", self._cb_odom, 10)
        node.create_subscription(Float32, "/PowerVoltage", self._cb_battery, 10)
        node.create_subscription(Bool, "/chassis_enabled", self._cb_enabled, 10)
        node.create_subscription(LaserScan, "/scan", self._cb_scan,
                                 qos_profile_sensor_data)
        node.create_subscription(OccupancyGrid, "/map", self._cb_map, map_qos)

        self._pub_cmd = node.create_publisher(Twist, "/cmd_vel", 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)
        self._nav_client = ActionClient(node, NavigateToPose,
                                        "navigate_to_pose")

        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(node,), daemon=True,
            name="rescue-gateway-spin",
        )
        self._spin_thread.start()

    # ---- 订阅回调（spin 线程内执行）----

    def _cb_odom(self, msg) -> None:
        import math

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # 里程累计：用位姿增量近似，满足控制台显示精度即可
        if self._odo_prev is not None:
            self._odo += math.hypot(p.x - self._odo_prev[0],
                                     p.y - self._odo_prev[1])
        self._odo_prev = (p.x, p.y)
        self._pose = {"x": p.x, "y": p.y, "theta": yaw}
        t = msg.twist.twist
        self._twist = {"vx": t.linear.x, "vy": t.linear.y, "wz": t.angular.z}

    def _cb_battery(self, msg) -> None:
        self._battery = float(msg.data)

    def _cb_enabled(self, msg) -> None:
        self._enabled = bool(msg.data)

    def _cb_scan(self, msg) -> None:
        self._scan = msg

    def _cb_map(self, msg) -> None:
        info = msg.info
        self._map_meta = (
            info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y,
        )
        self._map_flat = list(msg.data)
        self._map_dirty = True

    # ---- action 回调（spin 线程内执行）----

    def _goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._nav_state = "rejected"
            self._goal = None
            return
        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        # GoalStatus 枚举：3=执行中 4=SUCCEEDED 5=CANCELED 6=ABORTED 等
        try:
            status = future.result().status
        except Exception:
            status = -1
        self._goal_handle = None
        self._goal = None
        if status == 4:
            self._nav_state = "reached"
        elif status == 5:
            self._nav_state = "canceled"
        elif status == 3:
            self._nav_state = "navigating"
        else:
            self._nav_state = "failed"

    # ---- BaseBridge 接口（事件循环内调用）----

    def tick(self, dt: float) -> None:
        # 看门狗：wheeltec_robot_node 自身也有 0.5s 超时停车，
        # 这里是网关侧的第二层保护（浏览器断开/卡顿同样触发）。
        if self._manual and time.monotonic() - self._last_cmd > _CMD_VEL_TIMEOUT:
            self._publish_cmd(0.0, 0.0, 0.0)
            self._manual = False

    def telemetry(self) -> dict:
        import math

        pose = dict(self._pose)
        frame = "odom"
        try:
            if self._tf_buffer.can_transform(
                    "map", "base_footprint", self._rclpy.time.Time()):
                t = self._tf_buffer.lookup_transform(
                    "map", "base_footprint", self._rclpy.time.Time())
                q = t.transform.rotation
                pose = {
                    "x": t.transform.translation.x,
                    "y": t.transform.translation.y,
                    "theta": math.atan2(
                        2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
                }
                frame = "map"
        except Exception:
            pass  # TF 暂不可用时回退 odom 位姿

        scan = None
        if self._scan is not None:
            s = self._scan
            # 激光系 -> 机器人系：加安装朝向偏移；非有限值用 -1 表示超量程
            scan = {
                "angle_min": round(s.angle_min + self._laser_offset, 4),
                "angle_max": round(s.angle_max + self._laser_offset, 4),
                "range_max": round(float(s.range_max), 2),
                "ranges": [
                    -1 if not math.isfinite(r) else round(r, 2)
                    for r in s.ranges
                ],
            }

        battery = self._battery
        return {
            "type": "telemetry",
            "bridge": self.name,
            "pose": {k: round(v, 3) for k, v in pose.items()},
            "pose_frame": frame,
            "twist": {k: round(v, 3) for k, v in self._twist.items()},
            "battery": None if battery is None else round(battery, 2),
            "battery_low": battery is not None and battery <= _BATTERY_MIN,
            "chassis_enabled": self._enabled,
            "odometry": round(self._odo, 1),
            "nav_state": self._nav_state,
            "nav_goal": ({"x": self._goal[0], "y": self._goal[1]}
                         if self._goal else None),
            "manual": self._manual,
            "scan": scan,
        }

    def map_message(self, force: bool = False) -> Optional[dict]:
        if self._map_flat is None or self._map_meta is None:
            return None
        if not self._map_dirty and force:
            return self._map_cache
        if not self._map_dirty:
            return None
        width, height, res, ox, oy = self._map_meta
        self._map_dirty = False
        self._map_cache = {
            "type": "map",
            "width": width,
            "height": height,
            "resolution": res,
            "origin": [ox, oy],
            "rle": _rle_encode(self._map_flat),
        }
        return self._map_cache

    def _publish_cmd(self, vx: float, vy: float, wz: float) -> None:
        msg = self._Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self._pub_cmd.publish(msg)

    def cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        """手动遥控入口。非零指令优先取消 Nav2 目标（人工接管优先）。"""
        if self._closed:
            return
        self._publish_cmd(vx, vy, wz)
        self._last_cmd = time.monotonic()
        self._manual = abs(vx) + abs(vy) + abs(wz) > 1e-6
        if self._manual and self._goal_handle is not None:
            self.cancel_nav()

    def nav_goal(self, x: float, y: float) -> None:
        if self._closed:
            return
        if not self._nav_client.server_is_ready():
            self._nav_state = "nav2_unavailable"
            return
        from geometry_msgs.msg import PoseStamped

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        goal_msg = self._NavigateToPose.Goal()
        goal_msg.pose = pose
        self._goal = (float(x), float(y))
        self._nav_state = "navigating"
        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_cb)

    def cancel_nav(self) -> None:
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._goal = None
        self._goal_handle = None
        self._nav_state = "idle"
        self._publish_cmd(0.0, 0.0, 0.0)

    def close(self) -> None:
        """服务退出时清理 rclpy 资源（spin 线程为守护线程，随进程结束）。"""
        if self._closed:
            return
        self._closed = True
        try:
            self._nav_client.destroy()
        except Exception:
            pass
        try:
            self._node.destroy_node()
        except Exception:
            pass
        try:
            self._rclpy.shutdown()
        except Exception:
            pass


def _rle_encode(flat: list) -> list:
    """占用栅格行程编码：[[count, value], ...]。

    输入为一维栅格（-1 未知 / 0 自由 / 100 占用），未知区域占大头，
    RLE 后体积远小于原始数组。
    """
    rle: list[list[int]] = []
    prev = flat[0]
    count = 0
    for v in flat:
        if v == prev:
            count += 1
        else:
            rle.append([count, prev])
            prev = v
            count = 1
    rle.append([count, prev])
    return rle


def create_bridge() -> BaseBridge:
    """创建桥接实例。当前仅有面向目标机 ROS2 的实现。

    构造 RosBridge 会初始化 rclpy 节点并启动 spin 线程；若环境未 source
    ROS2，此处会抛出带中文说明的 RuntimeError，由调用方（app.py）在启动时
    直接暴露给运维人员。
    """
    return RosBridge()
