#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""KCF 跟随控制律（与 ROS 解耦，供两种跟随模式复用）。

业务目的：
  把 KCF 输出的「目标距离 + 目标在画面中的像素横坐标」转换为底盘的
  线速度与角速度：距离环用 PD 逼近期望跟车距离，角度环用 PD 把目标
  拉回画面中心。

为什么要抽出来：
  工程里有两种跟随模式需要同一套控制律——持续跟随（kcf_follow.py）与
  两阶段融合跟随（follow_target_server.py）。若不共用，两处参数与公式
  会各自漂移，调好的参数无法互通。

数据约定（KCF 节点 kcf/track 的输出，借用 Twist 消息承载）：
  linear.x   目标距离（米），<=0 表示无有效观测
  angular.z  目标在彩色图中的像素横坐标，<=0 表示无有效观测

用法：
  ctrl = FollowController(distance_kp=..., ...)
  linear, angular = ctrl.update(distance, pixel_x)
  切换控制阶段前调用 ctrl.reset()，避免微分项帶着上一阶段的誤差跳变。
"""


class FollowController:
    """KCF 跟踪结果的 PD 控制器。不含 ROS 依赖，便于单独验证与复用。"""

    def __init__(
        self,
        distance_kp=0.1,
        distance_kd=0.5,
        target_distance=1.2,
        angle_kp=0.002,
        angle_kd=0.001,
        target_pixel_x=320.0,
        max_linear_speed=0.3,
        max_angular_speed=0.4,
    ):
        self.distance_kp = distance_kp
        self.distance_kd = distance_kd
        self.target_distance = target_distance
        self.angle_kp = angle_kp
        self.angle_kd = angle_kd
        self.target_pixel_x = target_pixel_x
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.reset()

    def reset(self):
        """清空微分项歷史。

        阶段切换（例如从 Nav2 导航切到视觉伺服）时必须调用：否则上一阶段
        残留的誤差会作为 D 项输入，在切换瞬间输出一个速度跳变。
        """
        self._last_distance_error = 0.0
        self._last_angle_error = 0.0
        self._has_history = False

    @staticmethod
    def clamp(value, limit):
        """限幅。limit 取绝对值，避免传入负的限速导致区间反向。"""
        return max(-abs(limit), min(abs(limit), value))

    def update(self, distance, target_pixel_x):
        """根据一次观测计算速度，返回 (线速度, 角速度)。

        观测无效（距离或像素非正）时返回 (0.0, 0.0) 并清空微分項，
        调用方据此停车——目标丢失时继续按旧誤差驱动底盘是危险的。
        """
        if distance <= 0.0 or target_pixel_x <= 0.0:
            self._has_history = False
            return 0.0, 0.0

        distance_error = self.target_distance - distance
        angle_error = self.target_pixel_x - target_pixel_x

        # 首个有效观测没有歷史誤差：D 项按 0 处理，否则首帧会输出大跳变
        if not self._has_history:
            self._last_distance_error = distance_error
            self._last_angle_error = angle_error
            self._has_history = True

        linear = (
            -self.distance_kp * distance_error
            - self.distance_kd * (distance_error - self._last_distance_error)
        )
        angular = (
            self.angle_kp * angle_error
            + self.angle_kd * (angle_error - self._last_angle_error)
        )

        self._last_distance_error = distance_error
        self._last_angle_error = angle_error
        return (
            self.clamp(linear, self.max_linear_speed),
            self.clamp(angular, self.max_angular_speed),
        )
