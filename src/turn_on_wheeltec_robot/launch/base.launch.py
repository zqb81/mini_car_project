#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


SUPPORTED_MODELS = {
    "mini_4wd",
    "mini_akm",
    "mini_diff",
    "mini_mec",
    "mini_omni",
}


def launch_setup(context):
    # 在生成节点前解析并校验车型，避免因 URDF 缺失导致节点启动后才出现难以定位的 TF 故障。
    package_share = Path(get_package_share_directory("turn_on_wheeltec_robot"))
    model = LaunchConfiguration("model").perform(context)
    if model not in SUPPORTED_MODELS:
        raise RuntimeError(
            f"不支持车型 {model}，可选值为：{', '.join(sorted(SUPPORTED_MODELS))}"
        )

    urdf_path = package_share / "urdf" / f"{model}_robot.urdf"
    if not urdf_path.is_file():
        raise RuntimeError(f"找不到机器人模型：{urdf_path}")

    robot_description = urdf_path.read_text(encoding="utf-8")
    bridge_config = str(package_share / "config" / "wheeltec_bridge.yaml")

    # 底盘节点负责 odom 到 base_footprint，robot_state_publisher 负责 URDF 中的关节 TF；两者职责不可重叠。
    return [
        Node(
            package="turn_on_wheeltec_robot",
            executable="wheeltec_robot_node",
            name="wheeltec_robot",
            output="screen",
            parameters=[
                bridge_config,
                {
                    "serial_port": LaunchConfiguration("serial_port"),
                    "baud_rate": ParameterValue(
                        LaunchConfiguration("baud_rate"), value_type=int
                    ),
                    "odom_frame": LaunchConfiguration("odom_frame"),
                    "base_frame": LaunchConfiguration("base_frame"),
                    "imu_frame": LaunchConfiguration("imu_frame"),
                    "publish_odom_tf": ParameterValue(
                        LaunchConfiguration("publish_odom_tf"), value_type=bool
                    ),
                    "cmd_vel_timeout": ParameterValue(
                        LaunchConfiguration("cmd_vel_timeout"), value_type=float
                    ),
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool
                    ),
                },
            ],
            # 底盘节点内部硬编码订阅 "cmd_vel"。接入 twist_mux 后底盘只应接收
            # 仲裁输出，因此在这里做话题重映射，避免修改 C++ 逻辑。
            # 默认保持 "cmd_vel"，行为与接入仲裁前完全一致。
            remappings=[
                ("cmd_vel", LaunchConfiguration("cmd_vel_topic")),
            ],
        ),
        # IMU 安装位姿当前为默认值，实车安装位置或朝向变化时必须同步标定此静态变换。
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description,
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool
                    ),
                }
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_footprint_to_base_link",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", LaunchConfiguration("base_frame"),
                "--child-frame-id", "base_link",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_imu",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0.08",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", LaunchConfiguration("base_frame"),
                "--child-frame-id", LaunchConfiguration("imu_frame"),
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_camera",
            condition=IfCondition(LaunchConfiguration("publish_camera_tf")),
            arguments=[
                "--x", LaunchConfiguration("camera_x"),
                "--y", LaunchConfiguration("camera_y"),
                "--z", LaunchConfiguration("camera_z"),
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", LaunchConfiguration("base_frame"),
                "--child-frame-id", LaunchConfiguration("camera_frame"),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value="mini_mec"),
            DeclareLaunchArgument(
                "serial_port", default_value="/dev/wheeltec_controller"
            ),
            DeclareLaunchArgument("baud_rate", default_value="115200"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("imu_frame", default_value="imu_link"),
            DeclareLaunchArgument("publish_odom_tf", default_value="true"),
            DeclareLaunchArgument("cmd_vel_timeout", default_value="0.5"),
            # 底盘接收速度指令的话题。单独运行底盘时保持默认 "cmd_vel"；
            # 接入 twist_mux 后由上层传入仲裁输出话题（如 /cmd_vel_muxed），
            # 使底盘只接受唯一一路经过仲裁的指令。
            DeclareLaunchArgument("cmd_vel_topic", default_value="cmd_vel"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("publish_camera_tf", default_value="true"),
            DeclareLaunchArgument("camera_frame", default_value="camera_link"),
            DeclareLaunchArgument("camera_x", default_value="0.12"),
            DeclareLaunchArgument("camera_y", default_value="0.0"),
            DeclareLaunchArgument("camera_z", default_value="0.15"),
            OpaqueFunction(function=launch_setup),
        ]
    )
