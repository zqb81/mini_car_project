#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """启动 Astra S，并输出已对齐的 RGB 与深度图给 RTAB-Map 使用。"""
    camera_parameters = {
        "camera_name": "camera",
        "color_width": 640,
        "color_height": 480,
        "color_fps": 30,
        "enable_color": True,
        "enable_ir": False,
        "depth_width": 640,
        "depth_height": 480,
        "depth_fps": 30,
        "enable_depth": True,
        # RTAB-Map 的 RGB-D 同步器使用彩色相机内参，因此深度必须对齐到彩色图。
        "depth_align": True,
        "color_depth_synchronization": True,
        "depth_scale": 1,
        "serial_number": "",
        "number_of_devices": 1,
        "publish_tf": True,
        "tf_publish_rate": 10.0,
        "reconnect_timeout": 6.0,
    }

    return LaunchDescription(
        [
            ComposableNodeContainer(
                name="astra_camera_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                output="screen",
                composable_node_descriptions=[
                    ComposableNode(
                        package="astra_camera",
                        plugin="astra_camera::OBCameraNodeFactory",
                        namespace="camera",
                        name="camera",
                        parameters=[camera_parameters],
                    )
                ],
            )
        ]
    )
