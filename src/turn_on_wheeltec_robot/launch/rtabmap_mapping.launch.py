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
    # 纯建图入口：生成地图数据库但不启动 Nav2，适合首次采集环境或重建已有地图。
    package_share = get_package_share_directory("turn_on_wheeltec_robot")
    base_launch = PythonLaunchDescriptionSource(
        os.path.join(package_share, "launch", "base.launch.py")
    )

    rgb_topic = LaunchConfiguration("rgb_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")

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
            DeclareLaunchArgument(
                "database_path", default_value="~/.ros/mini_car_rtabmap.db"
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
                    ("rgb/image", rgb_topic),
                    ("depth/image", depth_topic),
                    ("rgb/camera_info", camera_info_topic),
                    ("rgbd_image", "rgbd_image"),
                ],
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                namespace="rtabmap",
                name="rtabmap",
                output="screen",
                # -d 会删除同路径旧数据库，防止新建地图混入旧环境节点；需要保留旧图时改用新的数据库路径。
                arguments=["-d"],
                parameters=[
                    {
                        "database_path": LaunchConfiguration("database_path"),
                        "frame_id": LaunchConfiguration("base_frame"),
                        "subscribe_rgbd": True,
                        "subscribe_scan": True,
                        "approx_sync": True,
                        "qos_scan": 2,
                        "qos_odom": 1,
                        "queue_size": 20,
                        # 平面底盘仅估计 x、y 与偏航角，以匹配后续 2D 激光导航的坐标约束。
                        "Reg/Force3DoF": "true",
                        "Reg/Strategy": "1",
                        "Grid/FromDepth": "false",
                        "RGBD/NeighborLinkRefining": "true",
                        "RGBD/ProximityBySpace": "true",
                        "RGBD/AngularUpdate": "0.05",
                        "RGBD/LinearUpdate": "0.05",
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                    }
                ],
                remappings=[
                    ("rgbd_image", "rgbd_image"),
                    ("scan", scan_topic),
                    ("odom", odom_topic),
                    ("map", "/map"),
                ],
            ),
        ]
    )
