#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动救援目标检测节点。

前置条件：
  1. Astra Pro 相机已启动（slam_navigation.launch.py 默认会启动），
     且 RGB、深度、内参三个话题都在发布；
  2. TF 链路 map -> odom -> base_footprint -> camera 完整（RTAB-Map 在建图时
     提供 map -> odom），否则无法把检测结果变换到 map 系；
  3. 已安装 ultralytics（见本包 requirements.txt）。

本 launch 只做感知，不启动任何会下发速度的控制器。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value="yolov8n.pt",
                description="YOLO 权重文件；nano 版在 CPU 上最轻",
            ),
            DeclareLaunchArgument(
                "target_classes",
                default_value="person",
                description="要检测的 COCO 类别名，多个用逗号分隔（如 person,backpack）",
            ),
            DeclareLaunchArgument(
                "conf_threshold", default_value="0.5",
                description="置信度阈值，低于此值的检测被丢弃",
            ),
            DeclareLaunchArgument(
                "min_interval", default_value="0.5",
                description="两次检测的最小间隔（秒）；树莓派上用于限流，0.5 即 2Hz",
            ),
            DeclareLaunchArgument(
                "target_frame", default_value="map",
                description="输出目标位姿所在的坐标系",
            ),
            DeclareLaunchArgument(
                "max_depth", default_value="8.0",
                description="有效测距上限（米），Astra 远距噪声大，超出不采信",
            ),
            DeclareLaunchArgument(
                "publish_debug_image", default_value="true",
                description="是否发布画了检测框的调试图像，便于 RViz 查看",
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
                        "publish_debug_image": LaunchConfiguration(
                            "publish_debug_image"
                        ),
                    }
                ],
            ),
        ]
    )
