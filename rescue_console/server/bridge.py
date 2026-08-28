# -*- coding: utf-8 -*-
"""桥接层：网关与数据源（模拟 / ROS2）之间的抽象。

业务目的：
  救援控制台网关不应关心数据来自模拟器还是真实小车。本文件定义统一桥接
  接口 BaseBridge，并提供两个实现：
    - MockBridge：本机 Demo 用，内置合成厂房环境、激光射线仿真、
      cmd_vel 看门狗与演示级导航控制器，不依赖 ROS2。
    - RosBridge：目标机（树莓派 + Ubuntu 22.04 + ROS2 Humble）用，通过
      rclpy 订阅 /odom、/PowerVoltage、/chassis_enabled、/scan、/map，
      发布 /cmd_vel，导航目标走 Nav2 NavigateToPose action。
  通过环境变量 RESCUE_BRIDGE=mock|ros 切换，默认 mock。

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
# 合成厂房环境（仅 MockBridge 使用）
# 地图 20m x 16m，分辨率 0.05m/格。坐标系：原点在左下角，x 向右，y 向上，
# theta 为弧度、逆时针为正（与 ROS 一致）。
# ---------------------------------------------------------------------------
_MAP_W_M = 20.0
_MAP_H_M = 16.0
_MAP_RES = 0.05
_MAP_COLS = int(_MAP_W_M / _MAP_RES)
_MAP_ROWS = int(_MAP_H_M / _MAP_RES)

# 障碍物列表：矩形（x0,y0,x1,y1），单位米
_WALLS = [
    (0.0, 0.0, 20.0, 0.2),    # 下外墙
    (0.0, 15.8, 20.0, 16.0),  # 上外墙
    (0.0, 0.0, 0.2, 16.0),    # 左外墙
    (19.8, 0.0, 20.0, 16.0),  # 右外墙
    (5.0, 3.0, 5.4, 8.0),     # 立柱 A
    (10.0, 6.0, 10.4, 11.0),  # 立柱 B
    (14.0, 2.0, 14.4, 7.0),   # 立柱 C
    (3.0, 11.0, 8.0, 11.4),   # 隔墙 1
    (12.0, 11.0, 17.0, 11.4), # 隔墙 2
    (8.0, 2.0, 12.0, 2.4),    # 货架
]

_SCAN_RAYS = 240          # 模拟激光射线数（真实 A1M8 约 800 点/帧）
_SCAN_RANGE_MAX = 12.0    # A1M8 量程 12m
_CMD_VEL_TIMEOUT = 0.5    # 手动指令超时（秒），与 ROS2 桥接看门狗一致
_MAX_LIN = 0.6            # 演示限速（米/秒），远低于实车能力，安全优先
_MAX_ANG = 1.5            # 演示最大角速度（弧度/秒）
_BATTERY_V0 = 12.6        # 满电电压（3S 锂电）
_BATTERY_MIN = 10.0       # 低电量阈值（伏），低于此值模拟停车返航提示


def _build_known_grid() -> list[list[int]]:
    """把矩形障碍物栅格化为真值地图。1 表示占用，0 表示自由。"""
    grid = [[0] * _MAP_COLS for _ in range(_MAP_ROWS)]
    for x0, y0, x1, y1 in _WALLS:
        c0 = int(x0 / _MAP_RES)
        c1 = int(x1 / _MAP_RES)
        r0 = int(y0 / _MAP_RES)
        r1 = int(y1 / _MAP_RES)
        for r in range(max(0, r0), min(_MAP_ROWS, r1 + 1)):
            for c in range(max(0, c0), min(_MAP_COLS, c1 + 1)):
                grid[r][c] = 1
    return grid


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


class MockBridge(BaseBridge):
    """模拟桥接：不依赖 ROS2 的本机演示数据源。

    行为说明：
    - 激光仿真对真值地图做 DDA 射线步进，同时把走过的格子标为自由、
      命中点标为占用，写入 explored 栅格，模拟 RTAB-Map 实时建图中
      “探索区域逐渐扩大”的视觉效果。
    - cmd_vel 指令带 0.5 秒看门狗：超时自动置零，复刻 ROS2 桥接的安全
      行为，浏览器断开或卡顿时小车会自动停下。
    - 手动遥控优先：收到任何非零手动指令即取消自动导航，符合
      “人工接管优先”的救援安全原则。
    - 导航控制器是演示级 P 控制器（无真正避障规划），仅用于展示
      “点击地图下发目标 -> 小车自动行驶”的交互链路。
    """

    name = "mock"

    def __init__(self) -> None:
        self._known = _build_known_grid()
        # explored 栅格：-1 未知 / 0 自由 / 100 占用（与 OccupancyGrid 一致）
        self._explored = [[-1] * _MAP_COLS for _ in range(_MAP_ROWS)]
        self._x = 2.5
        self._y = 2.5
        self._theta = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._last_cmd_time = 0.0
        self._manual = False          # 是否处于手动接管状态
        self._goal: Optional[tuple[float, float]] = None
        self._nav_state = "idle"      # idle / navigating / reached / blocked
        self._battery = _BATTERY_V0
        self._odo = 0.0               # 累计里程（米）
        self._ranges = [float("inf")] * _SCAN_RAYS
        self._map_dirty = True        # 地图是否有未推送的更新
        self._last_map: Optional[dict] = None  # 缓存最近一帧地图，供新客户端补发

    # ---- 内部工具 ---------------------------------------------------------

    def _occupied(self, x: float, y: float) -> bool:
        """世界坐标是否命中障碍。越界视为占用，防止穿墙。"""
        c = int(x / _MAP_RES)
        r = int(y / _MAP_RES)
        if c < 0 or c >= _MAP_COLS or r < 0 or r >= _MAP_ROWS:
            return True
        return self._known[r][c] == 1

    def _raycast(self, ang: float) -> float:
        """沿 ang 方向做固定步长射线步进，返回命中距离。

        步长取 0.5 * 分辨率，兼顾精度与 CPU 开销（240 射线 * 10Hz）。
        命中时同时更新 explored 栅格，模拟实时建图。
        """
        step = _MAP_RES * 0.5
        dx = math.cos(ang) * step
        dy = math.sin(ang) * step
        x, y = self._x, self._y
        dist = 0.0
        while dist < _SCAN_RANGE_MAX:
            x += dx
            y += dy
            dist += step
            if self._occupied(x, y):
                self._mark(x, y, 100)
                return dist
            self._mark(x, y, 0)
        return _SCAN_RANGE_MAX

    def _mark(self, x: float, y: float, value: int) -> None:
        c = int(x / _MAP_RES)
        r = int(y / _MAP_RES)
        if 0 <= c < _MAP_COLS and 0 <= r < _MAP_ROWS:
            if self._explored[r][c] != value:
                self._explored[r][c] = value
                self._map_dirty = True

    def _collide(self, nx: float, ny: float) -> bool:
        """以机器人中心近似做碰撞检测（演示级，忽略底盘半径）。"""
        return self._occupied(nx, ny)

    # ---- BaseBridge 接口 --------------------------------------------------

    def tick(self, dt: float) -> None:
        now = time.monotonic()

        # 手动指令看门狗：超时未收到新指令则速度归零
        if self._manual and now - self._last_cmd_time > _CMD_VEL_TIMEOUT:
            self._vx = self._vy = self._wz = 0.0
            self._manual = False

        # 演示级导航控制器：P 控制器，前方 0.6m 内有障碍则停车
        if self._goal is not None:
            gx, gy = self._goal
            dx, dy = gx - self._x, gy - self._y
            dist = math.hypot(dx, dy)
            if dist < 0.3:
                self._goal = None
                self._nav_state = "reached"
                self._vx = self._vy = self._wz = 0.0
            else:
                ang_err = math.atan2(dy, dx) - self._theta
                ang_err = math.atan2(math.sin(ang_err), math.cos(ang_err))
                self._wz = max(-1.0, min(1.0, 2.0 * ang_err))
                front = min(
                    self._ranges[i]
                    for i in range(_SCAN_RAYS)
                    if abs(i - _SCAN_RAYS // 2) < 20
                )
                if front < 0.6:
                    self._vx = self._vy = 0.0
                    self._nav_state = "blocked"
                else:
                    self._vx = min(0.4, 0.6 * dist) * max(
                        0.0, math.cos(ang_err)
                    )
                    self._vy = 0.0

        # 低电量保护：低于阈值停止自动导航并提示
        if self._battery <= _BATTERY_MIN and self._goal is not None:
            self._goal = None
            self._nav_state = "idle"
            self._vx = self._vy = self._wz = 0.0

        # 位姿积分
        half = dt * 0.5
        mid_th = self._theta + self._wz * half
        nx = self._x + (self._vx * math.cos(mid_th) - self._vy * math.sin(mid_th)) * dt
        ny = self._y + (self._vx * math.sin(mid_th) + self._vy * math.cos(mid_th)) * dt
        if not self._collide(nx, ny):
            self._odo += math.hypot(nx - self._x, ny - self._y)
            self._x, self._y = nx, ny
        else:
            # 撞墙：停车并取消导航，避免演示中穿模
            self._vx = self._vy = self._wz = 0.0
            self._goal = None
            if self._nav_state == "navigating":
                self._nav_state = "blocked"
        self._theta = math.atan2(
            math.sin(self._theta + self._wz * dt),
            math.cos(self._theta + self._wz * dt),
        )

        # 激光扫描（同时完成 explored 栅格更新）
        for i in range(_SCAN_RAYS):
            ang = self._theta + math.pi + 2.0 * math.pi * i / _SCAN_RAYS
            self._ranges[i] = self._raycast(ang)

        # 电量按运动强度缓降，静止时极慢自放电
        drain = (abs(self._vx) + abs(self._vy) + abs(self._wz) * 0.5) * dt
        self._battery = max(9.5, self._battery - drain * 0.004 - dt * 1e-5)

    def telemetry(self) -> dict:
        return {
            "type": "telemetry",
            "bridge": self.name,
            "pose": {"x": round(self._x, 3), "y": round(self._y, 3),
                     "theta": round(self._theta, 3)},
            "twist": {"vx": round(self._vx, 3), "vy": round(self._vy, 3),
                      "wz": round(self._wz, 3)},
            "battery": round(self._battery, 2),
            "battery_low": self._battery <= _BATTERY_MIN,
            "chassis_enabled": self._battery > 9.5,
            "odometry": round(self._odo, 1),
            "nav_state": self._nav_state,
            "nav_goal": ({"x": self._goal[0], "y": self._goal[1]}
                         if self._goal else None),
            "manual": self._manual,
            "scan": {
                "angle_min": round(math.pi, 4),
                "angle_max": round(-math.pi, 4),
                "range_max": _SCAN_RANGE_MAX,
                # 无穷大在 JSON 中不合法，超出量程用 -1 表示（与 ROS 约定一致）
                "ranges": [(-1 if r == _SCAN_RANGE_MAX else round(r, 2))
                           for r in self._ranges],
            },
        }

    def map_message(self, force: bool = False) -> Optional[dict]:
        """返回 RLE 压缩的占用栅格；无更新且未强制时返回 None。

        explored 栅格大部分区域是连续的 -1（未知），行程编码（RLE）后
        通常只有几千字节，1Hz 推送对局域网带宽无压力。force=True 用于
        新客户端连接时补发缓存帧。
        """
        if not self._map_dirty and not (force and self._last_map):
            return None
        if not self._map_dirty and force:
            return self._last_map
        self._map_dirty = False
        flat = []
        for row in self._explored:
            flat.extend(row)
        self._last_map = {
            "type": "map",
            "width": _MAP_COLS,
            "height": _MAP_ROWS,
            "resolution": _MAP_RES,
            "origin": [0.0, 0.0],
            "rle": _rle_encode(flat),
        }
        return self._last_map

    def cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        """手动遥控入口。非零指令会取消自动导航（人工接管优先）。"""
        self._vx = max(-_MAX_LIN, min(_MAX_LIN, vx))
        self._vy = max(-_MAX_LIN, min(_MAX_LIN, vy))
        self._wz = max(-_MAX_ANG, min(_MAX_ANG, wz))
        self._last_cmd_time = time.monotonic()
        self._manual = abs(vx) + abs(vy) + abs(wz) > 1e-6
        if self._manual and self._goal is not None:
            self._goal = None
            self._nav_state = "idle"

    def nav_goal(self, x: float, y: float) -> None:
        if not (0 < x < _MAP_W_M and 0 < y < _MAP_H_M):
            return
        self._goal = (x, y)
        self._nav_state = "navigating"

    def cancel_nav(self) -> None:
        self._goal = None
        self._nav_state = "idle"
        self._vx = self._vy = self._wz = 0.0


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
                "需先 source /opt/ros/humble/setup.bash）。"
                "演示模式请设置 RESCUE_BRIDGE=mock。"
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
        # 看门狗：与 MockBridge 语义一致。wheeltec_robot_node 自身也有
        # 0.5s 超时停车，这里是网关侧的第二层保护。
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
    """按环境变量选择桥接实现，默认 mock（本机 Demo）。"""
    import os

    kind = os.environ.get("RESCUE_BRIDGE", "mock").lower()
    if kind == "ros":
        return RosBridge()
    return MockBridge()
