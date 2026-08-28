#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    base_launch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory("turn_on_wheeltec_robot"),
            "launch",
            "base.launch.py",
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_base", default_value="true"),
            DeclareLaunchArgument("model", default_value="mini_mec"),
            DeclareLaunchArgument(
                "serial_port", default_value="/dev/wheeltec_controller"
            ),
            DeclareLaunchArgument(
                "rgb_topic", default_value="/camera/color/image_raw"
            ),
            DeclareLaunchArgument(
                "depth_topic", default_value="/camera/depth/image_raw"
            ),
            DeclareLaunchArgument("show_window", default_value="true"),
            IncludeLaunchDescription(
                base_launch,
                condition=IfCondition(LaunchConfiguration("start_base")),
                launch_arguments={
                    "model": LaunchConfiguration("model"),
                    "serial_port": LaunchConfiguration("serial_port"),
                }.items(),
            ),
            Node(
                package="kcf_track",
                executable="kcf_node",
                name="kcf_tracker",
                output="screen",
                parameters=[
                    {
                        "rgb_topic": LaunchConfiguration("rgb_topic"),
                        "depth_topic": LaunchConfiguration("depth_topic"),
                        "show_window": ParameterValue(
                            LaunchConfiguration("show_window"), value_type=bool
                        ),
                    }
                ],
            ),
            Node(
                package="kcf_track",
                executable="kcf_follow.py",
                name="kcf_follower",
                output="screen",
                parameters=[
                    {
                        "distance_kp": 0.1,
                        "distance_kd": 0.5,
                        "target_distance": 1.2,
                        "angle_kp": 0.002,
                        "angle_kd": 0.001,
                        "target_pixel_x": 320.0,
                        "max_linear_speed": 0.3,
                        "max_angular_speed": 0.4,
                        "tracking_timeout": 0.5,
                    }
                ],
            ),
        ]
    )
