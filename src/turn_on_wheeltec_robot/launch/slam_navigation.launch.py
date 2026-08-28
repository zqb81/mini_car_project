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
    # 本入口将增量 SLAM 与 Nav2 同时启动：RTAB-Map 维护在线地图，Nav2 订阅 /map 并输出 /cmd_vel。
    package_share = get_package_share_directory("turn_on_wheeltec_robot")
    nav2_share = get_package_share_directory("nav2_bringup")
    base_launch = PythonLaunchDescriptionSource(
        os.path.join(package_share, "launch", "base.launch.py")
    )
    nav2_launch = PythonLaunchDescriptionSource(
        os.path.join(nav2_share, "launch", "navigation_launch.py")
    )
    lidar_launch = PythonLaunchDescriptionSource(
        os.path.join(package_share, "launch", "rplidar_a1.launch.py")
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_base", default_value="true"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
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
                "database_path", default_value="~/.ros/mini_car_slam.db"
            ),
            DeclareLaunchArgument(
                "nav2_params",
                default_value=os.path.join(package_share, "config", "nav2_params.yaml"),
            ),
            DeclareLaunchArgument(
                "rgb_topic", default_value="/camera/rgb/image_raw"
            ),
            DeclareLaunchArgument(
                "depth_topic", default_value="/camera/depth/image"
            ),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera/rgb/camera_info"
            ),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument(
                "lidar_port", default_value="/dev/wheeltec_lidar"
            ),
            DeclareLaunchArgument("laser_frame", default_value="laser"),
            DeclareLaunchArgument("laser_x", default_value="0.06"),
            DeclareLaunchArgument("laser_y", default_value="0.0"),
            DeclareLaunchArgument("laser_z", default_value="0.20"),
            DeclareLaunchArgument("laser_roll", default_value="0.0"),
            DeclareLaunchArgument("laser_pitch", default_value="0.0"),
            DeclareLaunchArgument("laser_yaw", default_value="3.14159"),
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
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            IncludeLaunchDescription(
                lidar_launch,
                condition=IfCondition(LaunchConfiguration("start_lidar")),
                launch_arguments={
                    "serial_port": LaunchConfiguration("lidar_port"),
                    "frame_id": LaunchConfiguration("laser_frame"),
                    "base_frame": LaunchConfiguration("base_frame"),
                    "laser_x": LaunchConfiguration("laser_x"),
                    "laser_y": LaunchConfiguration("laser_y"),
                    "laser_z": LaunchConfiguration("laser_z"),
                    "laser_roll": LaunchConfiguration("laser_roll"),
                    "laser_pitch": LaunchConfiguration("laser_pitch"),
                    "laser_yaw": LaunchConfiguration("laser_yaw"),
                }.items(),
            ),
            Node(
                package="rtabmap_sync",
                executable="rgbd_sync",
                namespace="rtabmap",
                name="rgbd_sync",
                output="screen",
                # RGB 与深度时间戳存在小偏差时使用近似同步；同步失败将不会生成供 SLAM 消费的 rgbd_image。
                parameters=[
                    {
                        "approx_sync": True,
                        "qos_image": 2,
                        "qos_camera_info": 2,
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                    }
                ],
                remappings=[
                    ("rgb/image", LaunchConfiguration("rgb_topic")),
                    ("depth/image", LaunchConfiguration("depth_topic")),
                    ("rgb/camera_info", LaunchConfiguration("camera_info_topic")),
                    ("rgbd_image", "rgbd_image"),
                ],
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                namespace="rtabmap",
                name="rtabmap",
                output="screen",
                # 在线 SLAM 不使用 -d，否则每次启动都会删除数据库；增量记忆使新观测持续写入该库。
                parameters=[
                    {
                        "database_path": LaunchConfiguration("database_path"),
                        "frame_id": LaunchConfiguration("base_frame"),
                        "subscribe_rgbd": True,
                        "subscribe_scan": True,
                        "approx_sync": True,
                        "qos_image": 2,
                        "qos_scan": 2,
                        "qos_odom": 1,
                        "queue_size": 20,
                        # 小车在平面行驶，限制为 3DoF 可降低姿态漂移并与 2D Nav2 代价地图保持一致。
                        "Reg/Force3DoF": "true",
                        "Reg/Strategy": "1",
                        "Grid/FromDepth": "false",
                        "RGBD/NeighborLinkRefining": "true",
                        "RGBD/ProximityBySpace": "true",
                        "RGBD/AngularUpdate": "0.05",
                        "RGBD/LinearUpdate": "0.05",
                        "Mem/IncrementalMemory": "true",
                        "Mem/InitWMWithAllNodes": "false",
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
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
                # Nav2 只负责规划和控制，不启动 map_server 或 AMCL，定位与地图唯一来源为 RTAB-Map。
                nav2_launch,
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": LaunchConfiguration("autostart"),
                    "params_file": LaunchConfiguration("nav2_params"),
                    "use_composition": "False",
                }.items(),
            ),
        ]
    )
