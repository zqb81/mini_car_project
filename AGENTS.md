# 智能体工作约束

本文件适用于整个仓库。开始任何修改前，必须先阅读 docs/AGENT_HANDOFF.md 和 README.md。

## 项目基线

- 默认目标平台是 Ubuntu 22.04、ROS2 Humble、aarch64。
- 默认车型是 mini_mec，底盘 MCU 为 STM32F103VET6。
- ROS2 工作空间在目标机上直接使用仓库根目录 ~/mini_car_ws，源码目录为 ~/mini_car_ws/src。
- 主运行入口是 src/turn_on_wheeltec_robot/launch/slam_navigation.launch.py。
- 旧 ROS1 XML launch 和脚本仅用于迁移对照，不得重新接入 ROS2 安装流程。

## 编码与文档

- 所有新文件必须使用 UTF-8。
- 新代码注释和项目文档必须使用简体中文。
- 注释应解释业务目的、输入输出、边界条件和非显然算法，禁止只翻译代码。
- 修改 legacy STM32 文件前先检测编码；不得在无验证的情况下批量改变旧源码编码或换行。

## 设备事实

- STM32 控制板：CP2102 序列号 0002，稳定路径 /dev/wheeltec_controller。
- RPLIDAR A1M8：CP2102 序列号 0001，稳定路径 /dev/wheeltec_lidar，115200，默认 Standard 模式。
- 深度相机：Orbbec Astra Pro。USB 2bc5:0403 是深度设备，2bc5:0502 是独立彩色 UVC 设备。
- Astra 驱动 astra_camera 与 astra_camera_msgs 来自随车资料，含专有 OpenNI2 二进制，不提交公共仓库。

## 安全约束

- 实车电机测试前必须明确提示车轮悬空或场地清空，并确保硬件使能/急停可用。
- 不得使用 sudo ros2 launch 绕过设备权限。
- 不得同时启动两个底盘、雷达、相机或 map -> odom 发布者。
- 不得把 ttyUSB0/ttyUSB1 当作永久设备身份；使用稳定 udev 软链接。
- 当前只有 ROS2 侧 cmd_vel 超时停车，STM32 独立通信看门狗仍未实现。
- slam_navigation.launch.py 只启动系统，不会自动移动；必须收到手动速度或 Nav2 目标才运动。

## 修改与验证

1. 修改前运行 git status --short --branch，保留用户已有改动。
2. ROS2 C++/launch/参数修改至少检查：
   - Python launch 可由 ast.parse 解析。
   - YAML 可解析。
   - package.xml 与 URDF 可解析。
   - git diff --check 无错误。
3. 目标机有 ROS2 环境时，优先运行 colcon build --symlink-install --parallel-workers 1。
4. 传感器改动按底盘、雷达、相机、SLAM、Nav2 的顺序分层验收，不要直接跳到整栈。
5. 未在目标机验证的行为必须在交付说明中明确标为待验证。

## Git

- 稳定分支为 main，远程为 https://gitee.com/qbz23/mini_car_project.git。
- 禁止强推、硬重置或删除用户未确认的数据。
- 外部驱动目录 src/astra_camera 和 src/astra_camera_msgs 已被忽略。
- 外部 rplidar_ros 当前由目标机单独克隆；不要无理由将第三方源码并入主仓库。

