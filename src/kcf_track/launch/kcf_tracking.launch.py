#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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
            # 跟随模式：两种模式互斥，不能同时启动——它们都会向同一个
            # 速度话题下发指令，同时运行会互相打架。
            #   continuous 常驻跟随：kcf_follower 一有目标就驱动底盘（原有行为）
            #   fusion     两阶段融合：仅启动 follow_target 动作服务器，
            #              由调用方下发目标触发（先 Nav2 导航再视觉伺服）
            #   none       只做目标跟踪发布 kcf/track，不驱动底盘
            DeclareLaunchArgument("follow_mode", default_value="continuous"),
            DeclareLaunchArgument(
                "cmd_vel_topic", default_value="cmd_vel_kcf",
                description="跟随速度输出话题，配合 twist_mux 仲裁使用",
            ),
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
                condition=IfCondition(
                    PythonExpression(
                        ["'", LaunchConfiguration("follow_mode"), "' == 'continuous'"]
                    )
                ),
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
                        "distance_tolerance": 0.15,
                        "pixel_tolerance": 35.0,
                        "settle_count": 5,
                        "tracking_timeout": 0.5,
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                    }
                ],
            ),
            Node(
                package="kcf_track",
                executable="follow_target_server.py",
                name="follow_target_server",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        ["'", LaunchConfiguration("follow_mode"), "' == 'fusion'"]
                    )
                ),
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
                        "distance_tolerance": 0.15,
                        "pixel_tolerance": 35.0,
                        "settle_count": 5,
                        "tracking_timeout": 0.5,
                        # 伺服频率需显著高于 twist_mux 的 timeout 倒数，
                        # 否则仲裁器会判定跟随源失效而降级回导航
                        "servo_rate": 20.0,
                        "nav2_server_timeout": 5.0,
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                    }
                ],
            ),
        ]
    )
