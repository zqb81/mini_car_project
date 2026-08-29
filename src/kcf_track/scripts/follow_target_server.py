#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""两阶段融合跟随服务器：Nav2 导航到目标附近 + KCF 视觉伺服逼近。

业务目的：
  单独用 KCF 直发速度会绕过 Nav2 的避障与代价地图；单独用 Nav2 又无法
  逼近视觉目标（目标位姿来自相机、且可能移动）。本节点把两者按阶段串起来：

    阶段 1（可选）staging  用 Nav2 NavigateToPose 走到目标大致位置，
                           这段路享有完整的全局规划与局部避障。
    阶段 2       servo     进入视觉伺服环，KCF 持续给出目标距离与像素
                           位置，由 PD 控制器驱动底盘逼近并保持距离。

  任一时刻只有一个控制器活跃（阶段1 是 Nav2、阶段2 是伺服环），因此不会
  与导航争抢 /cmd_vel——这是与「KCF 常驻直发速度」最本质的区别。

设计参考：OpenNav Docking 的分阶段思路（先导航到 staging pose，再进入
vision-control loop 由视觉持续精化位姿）。

线程模型：
  伺服环需要在 action 回调内长时间运行，同时订阅回调必须继续更新观测。
  因此动作与订阅都挂在 ReentrantCallbackGroup 上，主函数使用
  MultiThreadedExecutor；若用单线程执行器，回调内的等待会阻塞订阅回调，
  导致观测永远不更新。
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from kcf_control import FollowController
from kcf_track.action import FollowTarget

# 阶段取值（与 FollowTarget.action 的 feedback.phase 约定一致）
PHASE_PENDING = 0
PHASE_STAGING = 1
PHASE_SERVO = 2
PHASE_DONE = 3

# 错误码（与 FollowTarget.action 的 result.error_code 约定一致）
ERROR_NONE = 0
ERROR_NAV2_UNAVAILABLE = 1
ERROR_STAGING_FAILED = 2
ERROR_TARGET_LOST = 3
ERROR_TIMEOUT = 4
ERROR_CANCELED = 5


class FollowTargetServer(Node):
    """两阶段融合跟随的动作服务器。"""

    def __init__(self):
        super().__init__("follow_target_server")

        # ---- 控制参数（默认值与 kcf_follow.py 保持一致，便于复用调好的值）----
        self.declare_parameter("distance_kp", 0.1)
        self.declare_parameter("distance_kd", 0.5)
        self.declare_parameter("target_distance", 1.2)
        self.declare_parameter("angle_kp", 0.002)
        self.declare_parameter("angle_kd", 0.001)
        self.declare_parameter("target_pixel_x", 320.0)
        self.declare_parameter("max_linear_speed", 0.3)
        self.declare_parameter("max_angular_speed", 0.4)
        self.declare_parameter("distance_tolerance", 0.15)
        self.declare_parameter("pixel_tolerance", 35.0)
        self.declare_parameter("settle_count", 5)

        # ---- 运行参数 ----
        # 超过该时长未收到有效观测即判定目标丢失并停车退出
        self.declare_parameter("tracking_timeout", 0.5)
        # 伺服环频率。必须显著高于 twist_mux 的 timeout（默认 0.5s）倒数，
        # 否则仲裁器会认为跟随源已失效而降级回导航。
        self.declare_parameter("servo_rate", 20.0)
        self.declare_parameter("nav2_server_timeout", 5.0)
        # 速度输出话题：默认走仲裁通道，不直发全局 /cmd_vel
        self.declare_parameter("cmd_vel_topic", "cmd_vel_kcf")

        self.tracking_timeout = self.get_parameter("tracking_timeout").value
        self.servo_rate = self.get_parameter("servo_rate").value
        self.nav2_server_timeout = self.get_parameter("nav2_server_timeout").value

        # 最新一次有效观测，由订阅回调写入
        self._latest_distance = -1.0
        self._latest_pixel_x = -1.0
        self._last_valid_time = None
        self._active_nav_goal = None
        self._goal_running = False
        self._goal_reserved = False

        self._pub_cmd = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10
        )

        # 动作与订阅共用可重入回调组，保证伺服环运行期间观测仍能更新
        self._group = ReentrantCallbackGroup()
        self._sub_track = self.create_subscription(
            Twist,
            "kcf/track",
            self._cb_track,
            10,
            callback_group=self._group,
        )
        self._nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=self._group
        )
        self._server = ActionServer(
            self,
            FollowTarget,
            "follow_target",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._group,
        )
        self.get_logger().info("两阶段融合跟随服务器已就绪（follow_target）。")

    # ---- 参数与工具 ----------------------------------------------------

    def _make_controller(self, target_distance):
        """按当前参数构造控制律。目标距离允许由请求覆盖。"""
        return FollowController(
            distance_kp=self.get_parameter("distance_kp").value,
            distance_kd=self.get_parameter("distance_kd").value,
            target_distance=target_distance,
            angle_kp=self.get_parameter("angle_kp").value,
            angle_kd=self.get_parameter("angle_kd").value,
            target_pixel_x=self.get_parameter("target_pixel_x").value,
            max_linear_speed=self.get_parameter("max_linear_speed").value,
            max_angular_speed=self.get_parameter("max_angular_speed").value,
        )

    def _cb_track(self, msg):
        """缓存 KCF 最新观测。约定：linear.x=距离，angular.z=像素横坐标。"""
        distance = msg.linear.x
        pixel_x = msg.angular.z
        if distance <= 0.0 or pixel_x <= 0.0:
            return  # 无效观测不更新，交由超时逻辑处理
        self._latest_distance = distance
        self._latest_pixel_x = pixel_x
        self._last_valid_time = time.monotonic()

    def _publish_stop(self):
        self._pub_cmd.publish(Twist())

    def _on_goal(self, goal_request):
        # 单底盘只允许一个跟随动作，避免并发 goal 共享导航句柄和伺服输出。
        if self._goal_running or self._goal_reserved:
            return GoalResponse.REJECT
        self._goal_reserved = True
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    # ---- 阶段 1：Nav2 导航到 staging pose --------------------------------

    def _navigate_to_staging(self, goal_handle, feedback, staging_pose):
        """用 Nav2 走到目标大致位置。返回错误码（ERROR_NONE 表示成功）。"""
        self.get_logger().info("阶段 1：导航至 staging pose。")
        feedback.phase = PHASE_STAGING
        goal_handle.publish_feedback(feedback)

        if not self._nav_client.wait_for_server(
            timeout_sec=self.nav2_server_timeout
        ):
            self.get_logger().error("Nav2 动作服务不可用，无法执行 staging。")
            return ERROR_NAV2_UNAVAILABLE

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = staging_pose
        send_future = self._nav_client.send_goal_async(goal_msg)

        # 轮询等待接受结果：不能用 spin_until_future_complete，
        # 因为当前正处在 executor 的回调线程中，嵌套 spin 会出问题。
        while not send_future.done():
            if goal_handle.is_cancel_requested:
                self.get_logger().warning("staging 阶段被取消。")
                return ERROR_CANCELED
            time.sleep(0.05)

        nav_goal_handle = send_future.result()
        if nav_goal_handle is None or not nav_goal_handle.accepted:
            self.get_logger().error("Nav2 拒绝了 staging 目标。")
            return ERROR_STAGING_FAILED
        self._active_nav_goal = nav_goal_handle

        result_future = nav_goal_handle.get_result_async()
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                # 取消时同步取消 Nav2 目标，避免导航继续驱动底盘
                nav_goal_handle.cancel_goal_async()
                return ERROR_CANCELED
            time.sleep(0.05)

        status = result_future.result().status
        self._active_nav_goal = None
        if status != 4:  # GoalStatus.SUCCEEDED == 4
            self.get_logger().warning(f"Nav2 staging 未完成，状态码 {status}。")
            return ERROR_STAGING_FAILED
        return ERROR_NONE

    # ---- 阶段 2：视觉伺服环 ---------------------------------------------

    def _visual_servo(self, goal_handle, feedback, controller, servo_timeout):
        """视觉伺服环。返回 (错误码, 结束时的距离)。"""
        self.get_logger().info("阶段 2：进入视觉伺服环。")
        feedback.phase = PHASE_SERVO

        # 进入伺服前清空微分项，避免带着上一阶段的误差产生速度跳变
        controller.reset()

        period = 1.0 / self.servo_rate
        start = time.monotonic()
        final_distance = -1.0
        settled = 0

        while rclpy.ok():
            now = time.monotonic()

            if goal_handle.is_cancel_requested:
                self._publish_stop()
                return ERROR_CANCELED, final_distance

            if self._last_valid_time is None:
                # 从未获得观测：给一个宽限期等待 KCF 出结果
                if now - start > self.tracking_timeout:
                    self.get_logger().warning("伺服开始后始终未获得有效观测。")
                    self._publish_stop()
                    return ERROR_TARGET_LOST, final_distance
            elif now - self._last_valid_time > self.tracking_timeout:
                self.get_logger().warning("视觉伺服期间目标丢失，已停车。")
                self._publish_stop()
                return ERROR_TARGET_LOST, final_distance

            if servo_timeout > 0.0 and now - start > servo_timeout:
                self.get_logger().info("达到 servo_timeout，结束伺服。")
                self._publish_stop()
                return ERROR_TIMEOUT, final_distance

            # 单帧无效时发布零速但不退出：短暂遮挡不应立即放弃跟随
            if self._last_valid_time is None:
                linear, angular = 0.0, 0.0
            else:
                linear, angular = controller.update(
                    self._latest_distance, self._latest_pixel_x
                )
                final_distance = self._latest_distance
                distance_ok = abs(final_distance - controller.target_distance) <= self.get_parameter("distance_tolerance").value
                pixel_ok = abs(self._latest_pixel_x - self.get_parameter("target_pixel_x").value) <= self.get_parameter("pixel_tolerance").value
                settled = settled + 1 if distance_ok and pixel_ok else 0
                if settled >= int(self.get_parameter("settle_count").value):
                    self._publish_stop()
                    return ERROR_NONE, final_distance

            cmd = Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            self._pub_cmd.publish(cmd)

            feedback.phase = PHASE_SERVO
            feedback.current_distance = float(final_distance)
            feedback.elapsed_time = now - start
            goal_handle.publish_feedback(feedback)

            time.sleep(period)

        self._publish_stop()
        return ERROR_CANCELED, final_distance

    # ---- 动作执行 -------------------------------------------------------

    def _execute(self, goal_handle):
        request = goal_handle.request
        self._goal_reserved = False
        self._goal_running = True
        # 每个动作必须从当前目标重新等待观测，不能沿用上一次目标的缓存数据。
        self._latest_distance = -1.0
        self._latest_pixel_x = -1.0
        self._last_valid_time = None
        feedback = FollowTarget.Feedback()
        feedback.phase = PHASE_PENDING
        feedback.current_distance = -1.0
        feedback.elapsed_time = 0.0

        # 请求可覆盖期望距离，未指定则沿用服务端默认参数
        target_distance = request.target_distance
        if target_distance <= 0.0:
            target_distance = self.get_parameter("target_distance").value
        controller = self._make_controller(target_distance)

        error_code = ERROR_NONE
        final_distance = -1.0

        if request.use_staging_pose:
            error_code = self._navigate_to_staging(
                goal_handle, feedback, request.staging_pose
            )

        if error_code == ERROR_NONE:
            error_code, final_distance = self._visual_servo(
                goal_handle, feedback, controller, request.servo_timeout
            )

        # 无论成功失败都确保停车，避免动作结束时底盘仍在运动
        if self._active_nav_goal is not None:
            self._active_nav_goal.cancel_goal_async()
            self._active_nav_goal = None
        self._publish_stop()

        result = FollowTarget.Result()
        result.error_code = error_code
        result.success = error_code == ERROR_NONE
        result.final_distance = float(final_distance)

        feedback.phase = PHASE_DONE
        goal_handle.publish_feedback(feedback)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif error_code == ERROR_NONE:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        self._goal_running = False
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FollowTargetServer()
    # 必须用多线程执行器：伺服环在动作回调内长时间运行，
    # 单线程会阻塞订阅回调导致观测无法更新。
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        if node._active_nav_goal is not None:
            node._active_nav_goal.cancel_goal_async()
            node._active_nav_goal = None
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
