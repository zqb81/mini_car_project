#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port", default_value="/dev/wheeltec_lidar"
            ),
            DeclareLaunchArgument("serial_baudrate", default_value="115200"),
            DeclareLaunchArgument("frame_id", default_value="laser"),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            # mini_mec 随车 ROS1 配置的 A1 初始安装位姿，实车仍需复核。
            DeclareLaunchArgument("laser_x", default_value="0.06"),
            DeclareLaunchArgument("laser_y", default_value="0.0"),
            DeclareLaunchArgument("laser_z", default_value="0.20"),
            DeclareLaunchArgument("laser_roll", default_value="0.0"),
            DeclareLaunchArgument("laser_pitch", default_value="0.0"),
            DeclareLaunchArgument("laser_yaw", default_value="3.14159"),
            DeclareLaunchArgument("inverted", default_value="false"),
            DeclareLaunchArgument("angle_compensate", default_value="true"),
            DeclareLaunchArgument("scan_mode", default_value="Standard"),
            Node(
                package="rplidar_ros",
                executable="rplidar_node",
                name="rplidar_node",
                output="screen",
                parameters=[
                    {
                        "channel_type": "serial",
                        "serial_port": LaunchConfiguration("serial_port"),
                        "serial_baudrate": ParameterValue(
                            LaunchConfiguration("serial_baudrate"), value_type=int
                        ),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "inverted": ParameterValue(
                            LaunchConfiguration("inverted"), value_type=bool
                        ),
                        "angle_compensate": ParameterValue(
                            LaunchConfiguration("angle_compensate"), value_type=bool
                        ),
                        "scan_mode": LaunchConfiguration("scan_mode"),
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser",
                output="screen",
                arguments=[
                    "--x", LaunchConfiguration("laser_x"),
                    "--y", LaunchConfiguration("laser_y"),
                    "--z", LaunchConfiguration("laser_z"),
                    "--roll", LaunchConfiguration("laser_roll"),
                    "--pitch", LaunchConfiguration("laser_pitch"),
                    "--yaw", LaunchConfiguration("laser_yaw"),
                    "--frame-id", LaunchConfiguration("base_frame"),
                    "--child-frame-id", LaunchConfiguration("frame_id"),
                ],
            ),
        ]
    )
