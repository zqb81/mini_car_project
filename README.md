# Mini ROS 小车工程

本仓库同时维护 STM32 底盘固件与树莓派 ROS 1 上位机源码。树莓派通过 USB 串口设备 `/dev/wheeltec_controller`，以 115200 波特率连接 STM32 的 USART3。

## 目录说明

- `F103VET6_Mini小车_STM32源码_2022.01.06(霍尔编码器)`：STM32F103VET6 底盘固件，使用 Keil MDK 和 FreeRTOS。
- `src/turn_on_wheeltec_robot`：ROS 底盘通信、里程计、IMU、TF、导航和建图配置包。`wheeltec_robot_node` 订阅 `/cmd_vel`，向 STM32 发送速度帧，并发布 `/odom`、`/imu` 与电池电压。
- `src/kcf_track`：基于 OpenCV 的 KCF 视觉跟踪包，计算跟随速度并发布 `/cmd_vel`。

`src/turn_on_wheeltec_robot/launch/3d_mapping.launch` 和 `3d_navigation.launch` 保留了原始树莓派版配置；其余底盘包文件来自随车资料中的完整 `turn_on_wheeltec_robot` 功能包。

## 树莓派部署

在树莓派的 ROS 1 catkin 工作空间中执行以下操作。假定工作空间为 `~/catkin_ws`，且工程目录放在 `~/catkin_ws/src/mini_car`。

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch
```

首次使用前，应确认底盘串口设备指向正确。随车规则脚本位于 `src/turn_on_wheeltec_robot/scripts/wheeltec_udev.sh`，会创建 `/dev/wheeltec_controller`。执行前应使用 `lsusb` 和 `udevadm info` 核对控制板的 VID、PID、序列号，避免把雷达或其他 USB 串口错误绑定为底盘。

底盘节点参数位于 `launch/include/base_serial.launch`：串口默认 `/dev/wheeltec_controller`，波特率默认 `115200`。若不使用 udev 规则，可将该参数临时改为实际设备，例如 `/dev/ttyUSB0`。

## Git 工作流

建议让 `main` 始终保持可构建、可部署状态，日常修改在功能分支完成。

```bash
git switch -c feature/serial-watchdog
# 修改、构建与实车验证
git status
git add src/turn_on_wheeltec_robot
git commit -m "feat: 增加串口通信看门狗"
git switch main
git merge --no-ff feature/serial-watchdog
```

每次提交前至少检查以下内容：

```bash
git diff --check
git status
```

构建产物、Python 缓存、Keil 的对象文件和个人界面配置已由 `.gitignore` 排除。需要发布到树莓派时，使用带标签的提交记录版本，例如 `git tag -a v0.1.0 -m "树莓派 ROS 底盘通信整合版"`。
