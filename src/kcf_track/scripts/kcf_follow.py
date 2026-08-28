#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# 控制律与持续跟随逻辑分离，供 follow_target_server.py 复用同一套参数与公式
from kcf_control import FollowController


class KcfFollower(Node):
    """将 KCF 输出的目标距离和像素位置转换为底盘速度。"""

    def __init__(self):
        super().__init__("kcf_follower")
        self.declare_parameter("distance_kp", 0.1)
        self.declare_parameter("distance_kd", 0.5)
        self.declare_parameter("target_distance", 1.2)
        self.declare_parameter("angle_kp", 0.002)
        self.declare_parameter("angle_kd", 0.001)
        self.declare_parameter("target_pixel_x", 320.0)
        self.declare_parameter("max_linear_speed", 0.3)
        self.declare_parameter("max_angular_speed", 0.4)
        self.declare_parameter("tracking_timeout", 0.5)
        # 速度输出话题。默认不再是全局 /cmd_vel：接入 twist_mux 后跟随只应
        # 通过仲裁通道下发，避免与 Nav2、Web 遥操直接争抢底盘。
        # 不接仲裁时可用 -p cmd_vel_topic:=cmd_vel 回到旧行为。
        self.declare_parameter("cmd_vel_topic", "cmd_vel_kcf")

        self.distance_kp = self.get_parameter("distance_kp").value
        self.distance_kd = self.get_parameter("distance_kd").value
        self.target_distance = self.get_parameter("target_distance").value
        self.angle_kp = self.get_parameter("angle_kp").value
        self.angle_kd = self.get_parameter("angle_kd").value
        self.target_pixel_x = self.get_parameter("target_pixel_x").value
        self.max_linear_speed = self.get_parameter("max_linear_speed").value
        self.max_angular_speed = self.get_parameter("max_angular_speed").value
        self.tracking_timeout = self.get_parameter("tracking_timeout").value

        # 控制律本体（含 PD 参数与限速），与两阶段融合跟随共用
        self.controller = FollowController(
            distance_kp=self.distance_kp,
            distance_kd=self.distance_kd,
            target_distance=self.target_distance,
            angle_kp=self.angle_kp,
            angle_kd=self.angle_kd,
            target_pixel_x=self.target_pixel_x,
            max_linear_speed=self.max_linear_speed,
            max_angular_speed=self.max_angular_speed,
        )
        self.last_tracking_time = time.monotonic()
        self.stop_sent = True

        self.velocity_publisher = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10
        )
        self.tracking_subscription = self.create_subscription(
            Twist, "kcf/track", self.tracking_callback, 10
        )
        self.watchdog_timer = self.create_timer(0.1, self.check_tracking_timeout)

    def tracking_callback(self, tracking):
        linear_speed, angular_speed = self.controller.update(
            tracking.linear.x, tracking.angular.z
        )

        # 观测无效时控制律返回零速度，目标丢失即停车
        if linear_speed == 0.0 and angular_speed == 0.0:
            self.publish_stop()
            return

        command = Twist()
        command.linear.x = linear_speed
        command.angular.z = angular_speed
        self.velocity_publisher.publish(command)

        self.last_tracking_time = time.monotonic()
        self.stop_sent = False

    def check_tracking_timeout(self):
        if (
            not self.stop_sent
            and time.monotonic() - self.last_tracking_time >= self.tracking_timeout
        ):
            self.get_logger().warning("KCF 跟踪数据超时，已发布零速度。")
            self.publish_stop()

    def publish_stop(self):
        self.velocity_publisher.publish(Twist())
        self.stop_sent = True


def main(args=None):
    rclpy.init(args=args)
    node = KcfFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
