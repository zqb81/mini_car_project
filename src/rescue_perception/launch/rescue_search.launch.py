#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自主搜索：目标不在视野时主动探索未知区域，发现目标则让位给接近动作。

前置条件：
  1. 已安装 m-explore-ros2（explore_lite 的 ROS2 移植，本仓库不内置）：

       cd ~/mini_car_ws/src
       git clone https://github.com/robo-friends/m-explore-ros2.git
       cd ~/mini_car_ws
       rosdep install --from-paths src --ignore-src -r -y
       colcon build --packages-select explore_lite --symlink-install

  2. slam_navigation.launch.py 已在运行（RTAB-Map 发布 /map，Nav2 可用）；
  3. 目标检测与融合已运行（rescue_perception.launch.py），本节点依赖
     target_fusion 发布的 /rescue/fusion_state 判断有无目标。

安全说明：
  自主搜索会让机器人自行移动，属于高风险行为，因此**默认不自动启动**
  （auto_start=false）。需由操作员显式开启：

       ros2 topic pub --once /rescue/search_cmd std_msgs/msg/Bool "{data: true}"

  停止搜索：把 data 置 false。搜索期间仍受 Nav2 避障与 twist_mux 约束。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rescue_perception")
    default_explore_params = os.path.join(pkg_share, "config", "explore_params.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "explore_params", default_value=default_explore_params,
                description="explore_lite 参数文件（frontier 尺寸、超时等）",
            ),
            DeclareLaunchArgument(
                "auto_start", default_value="false",
                description="上电即开始搜索（高风险，务必确认场地清空）",
            ),
            DeclareLaunchArgument(
                "resume_delay", default_value="5.0",
                description="目标结束后延迟多少秒恢复搜索",
            ),
            DeclareLaunchArgument(
                "start_explore", default_value="true",
                description="是否启动 explore_lite；false 时只启动编排节点",
            ),

            # 探索：只负责找 frontier 并调用 Nav2，从不发布速度指令
            Node(
                package="explore_lite",
                executable="explore",
                name="explore",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_explore")),
                parameters=[LaunchConfiguration("explore_params")],
            ),

            # 编排：根据融合状态决定探索的开与关
            Node(
                package="rescue_perception",
                executable="search_coordinator",
                name="search_coordinator",
                output="screen",
                parameters=[
                    {
                        "auto_start": LaunchConfiguration("auto_start"),
                        "resume_delay": LaunchConfiguration("resume_delay"),
                    }
                ],
            ),
        ]
    )
