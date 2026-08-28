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
    # 定位导航入口：加载已有 RTAB-Map 数据库并关闭增量记忆，避免导航过程中修改基准地图。
    package_share = get_package_share_directory("turn_on_wheeltec_robot")
    nav2_share = get_package_share_directory("nav2_bringup")

    base_launch = PythonLaunchDescriptionSource(
        os.path.join(package_share, "launch", "base.launch.py")
    )
    nav2_launch = PythonLaunchDescriptionSource(
        os.path.join(nav2_share, "launch", "navigation_launch.py")
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_base", default_value="true"),
            DeclareLaunchArgument("model", default_value="mini_mec"),
            DeclareLaunchArgument(
                "serial_port", default_value="/dev/wheeltec_controller"
            ),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("publish_camera_tf", default_value="true"),
            DeclareLaunchArgument("camera_frame", default_value="camera_link"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "database_path", default_value="~/.ros/mini_car_rtabmap.db"
            ),
            DeclareLaunchArgument(
                "nav2_params",
                default_value=package_share + "/config/nav2_params.yaml",
            ),
            DeclareLaunchArgument(
                "rgb_topic", default_value="/camera/color/image_raw"
            ),
            DeclareLaunchArgument(
                "depth_topic", default_value="/camera/depth/image_raw"
            ),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera/color/camera_info"
            ),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            IncludeLaunchDescription(
                base_launch,
                condition=IfCondition(LaunchConfiguration("start_base")),
                launch_arguments={
                    "model": LaunchConfiguration("model"),
                    "serial_port": LaunchConfiguration("serial_port"),
                    "base_frame": LaunchConfiguration("base_frame"),
                    "odom_frame": LaunchConfiguration("odom_frame"),
                    "publish_camera_tf": LaunchConfiguration("publish_camera_tf"),
                    "camera_frame": LaunchConfiguration("camera_frame"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
            Node(
                package="rtabmap_sync",
                executable="rgbd_sync",
                namespace="rtabmap",
                name="rgbd_sync",
                output="screen",
                # 近似同步仅解决传感器时间戳的小偏差；相机坐标系和深度图仍必须几何对齐。
                parameters=[
                    {
                        "approx_sync": True,
                        "qos_image": 2,
                        "qos_camera_info": 2,
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                    }
                ],
                remappings=[
                    ("rgb/image", LaunchConfiguration("rgb_topic")),
                    ("depth/image", LaunchConfiguration("depth_topic")),
                    (
                        "rgb/camera_info",
                        LaunchConfiguration("camera_info_topic"),
                    ),
                    ("rgbd_image", "rgbd_image"),
                ],
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                namespace="rtabmap",
                name="rtabmap",
                output="screen",
                parameters=[
                    {
                        "database_path": LaunchConfiguration("database_path"),
                        "frame_id": LaunchConfiguration("base_frame"),
                        "subscribe_rgbd": True,
                        "subscribe_scan": True,
                        "approx_sync": True,
                        "qos_scan": 2,
                        "qos_odom": 1,
                        "Reg/Force3DoF": "true",
                        "Reg/Strategy": "1",
                        "Grid/FromDepth": "false",
                        # 定位模式读取已有节点作为工作记忆，不再向数据库新增环境节点。
                        "Mem/IncrementalMemory": "false",
                        "Mem/InitWMWithAllNodes": "true",
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                    }
                ],
                remappings=[
                    ("rgbd_image", "rgbd_image"),
                    ("scan", LaunchConfiguration("scan_topic")),
                    ("odom", LaunchConfiguration("odom_topic")),
                    ("map", "/map"),
                ],
            ),
            IncludeLaunchDescription(
                # 地图与 map 到 odom 变换由 RTAB-Map 提供，Nav2 只消费这些结果完成规划和控制。
                nav2_launch,
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "autostart": LaunchConfiguration("autostart"),
                    "params_file": LaunchConfiguration("nav2_params"),
                    "use_composition": "False",
                }.items(),
            ),
        ]
    )
