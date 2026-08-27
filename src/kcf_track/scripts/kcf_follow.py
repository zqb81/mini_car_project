#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


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

        self.distance_kp = self.get_parameter("distance_kp").value
        self.distance_kd = self.get_parameter("distance_kd").value
        self.target_distance = self.get_parameter("target_distance").value
        self.angle_kp = self.get_parameter("angle_kp").value
        self.angle_kd = self.get_parameter("angle_kd").value
        self.target_pixel_x = self.get_parameter("target_pixel_x").value
        self.max_linear_speed = self.get_parameter("max_linear_speed").value
        self.max_angular_speed = self.get_parameter("max_angular_speed").value
        self.tracking_timeout = self.get_parameter("tracking_timeout").value

        self.last_distance_error = 0.0
        self.last_angle_error = 0.0
        self.last_tracking_time = time.monotonic()
        self.stop_sent = True

        self.velocity_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.tracking_subscription = self.create_subscription(
            Twist, "kcf/track", self.tracking_callback, 10
        )
        self.watchdog_timer = self.create_timer(0.1, self.check_tracking_timeout)

    @staticmethod
    def clamp(value, limit):
        return max(-abs(limit), min(abs(limit), value))

    def tracking_callback(self, tracking):
        distance = tracking.linear.x
        target_pixel_x = tracking.angular.z

        if distance <= 0.0 or target_pixel_x <= 0.0:
            self.publish_stop()
            return

        distance_error = self.target_distance - distance
        angle_error = self.target_pixel_x - target_pixel_x

        linear_speed = (
            -self.distance_kp * distance_error
            - self.distance_kd * (distance_error - self.last_distance_error)
        )
        angular_speed = (
            self.angle_kp * angle_error
            + self.angle_kd * (angle_error - self.last_angle_error)
        )

        command = Twist()
        command.linear.x = self.clamp(linear_speed, self.max_linear_speed)
        command.angular.z = self.clamp(angular_speed, self.max_angular_speed)
        self.velocity_publisher.publish(command)

        self.last_distance_error = distance_error
        self.last_angle_error = angle_error
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
