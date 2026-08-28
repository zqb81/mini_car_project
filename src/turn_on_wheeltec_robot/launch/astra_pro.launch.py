#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """启动 Astra Pro 的 OpenNI 深度设备和独立 UVC 彩色设备。"""
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
        # Astra Pro 的彩色摄像头通过独立 UVC 接口输出。
        "uvc_camera.enable": True,
        "uvc_camera.format": "mjpeg",
        "uvc_camera.vid": 0,
        "uvc_camera.pid": 0,
        "uvc_camera.retry_count": 100,
        # 对齐与同步后，RTAB-Map 可使用彩色相机内参处理 RGB-D 数据。
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
                name="astra_pro_camera_container",
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
