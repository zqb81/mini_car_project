#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动完整救援感知链路：目标检测 + 检测/导航融合。

链路：
  相机 -> detect_target（检测+深度投影）-> target_fusion（分级决策）
       -> FollowTarget（Nav2 导航到目标附近 + 视觉伺服逼近）

前置条件：
  1. 相机与 SLAM 已运行，TF 链路 map -> odom -> base_footprint -> camera 完整；
  2. kcf_track 以 follow_mode:=fusion 启动（提供 follow_target action 服务）；
  3. twist_mux 已部署（slam_navigation.launch.py 默认启用）。

只需单独调试检测时改用 detect_target.launch.py。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # ---------------- 检测 ----------------
            DeclareLaunchArgument(
                "model", default_value="yolov8n.pt",
                description="YOLO 权重文件",
            ),
            DeclareLaunchArgument(
                "target_classes", default_value="person",
                description="COCO 类别名，多个用逗号分隔",
            ),
            DeclareLaunchArgument(
                "conf_threshold", default_value="0.25",
                description="检测侧置信度下限（低于此值不上报）",
            ),
            DeclareLaunchArgument(
                "min_interval", default_value="0.5",
                description="检测最小间隔（秒），树莓派限流用",
            ),
            DeclareLaunchArgument(
                "max_depth", default_value="8.0",
                description="有效测距上限（米）",
            ),
            DeclareLaunchArgument(
                "target_frame", default_value="map",
                description="检测结果输出坐标系",
            ),

            # ---------------- 融合决策 ----------------
            DeclareLaunchArgument(
                "auto_conf_threshold", default_value="0.75",
                description="高于此置信度且目标稳定后自动下发导航目标",
            ),
            DeclareLaunchArgument(
                "confirm_conf_threshold", default_value="0.40",
                description="此值以上进入待人工确认，以下丢弃",
            ),
            DeclareLaunchArgument(
                "auto_mode", default_value="true",
                description="false 时所有目标都需人工确认（调试/高风险环境建议关）",
            ),
            DeclareLaunchArgument(
                "min_stable_count", default_value="3",
                description="连续多少次检测位置相近才认定为稳定目标",
            ),
            DeclareLaunchArgument(
                "stability_radius", default_value="0.5",
                description="位置稳定判定半径（米）",
            ),
            DeclareLaunchArgument(
                "follow_distance", default_value="1.2",
                description="视觉伺服阶段与目标保持的距离（米）",
            ),
            DeclareLaunchArgument(
                "staging_distance", default_value="2.0",
                description="导航阶段停在目标前方的距离（米）",
            ),
            DeclareLaunchArgument(
                "servo_timeout", default_value="60.0",
                description="伺服阶段时长上限（秒）",
            ),
            DeclareLaunchArgument(
                "detection_timeout", default_value="2.0",
                description="无检测消息多久后清空稳定计数（秒）",
            ),
            DeclareLaunchArgument(
                "pending_timeout", default_value="15.0",
                description="待确认目标保留时长（秒）",
            ),
            DeclareLaunchArgument(
                "start_fusion", default_value="true",
                description="false 时只启动检测，不启动融合决策",
            ),

            Node(
                package="rescue_perception",
                executable="detect_target",
                name="detect_target",
                output="screen",
                parameters=[
                    {
                        "model": LaunchConfiguration("model"),
                        "target_classes": LaunchConfiguration("target_classes"),
                        "conf_threshold": LaunchConfiguration("conf_threshold"),
                        "min_interval": LaunchConfiguration("min_interval"),
                        "target_frame": LaunchConfiguration("target_frame"),
                        "max_depth": LaunchConfiguration("max_depth"),
                    }
                ],
            ),
            Node(
                package="rescue_perception",
                executable="target_fusion",
                name="target_fusion",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_fusion")),
                parameters=[
                    {
                        "auto_conf_threshold": LaunchConfiguration(
                            "auto_conf_threshold"
                        ),
                        "confirm_conf_threshold": LaunchConfiguration(
                            "confirm_conf_threshold"
                        ),
                        "auto_mode": LaunchConfiguration("auto_mode"),
                        "min_stable_count": LaunchConfiguration(
                            "min_stable_count"
                        ),
                        "stability_radius": LaunchConfiguration(
                            "stability_radius"
                        ),
                        "follow_distance": LaunchConfiguration(
                            "follow_distance"
                        ),
                        "staging_distance": LaunchConfiguration(
                            "staging_distance"
                        ),
                        "servo_timeout": LaunchConfiguration("servo_timeout"),
                        "detection_timeout": LaunchConfiguration("detection_timeout"),
                        "pending_timeout": LaunchConfiguration("pending_timeout"),
                    }
                ],
            ),
        ]
    )
