# Mini ROS 小车工程

这是一个同时维护 STM32 底盘固件与树莓派 ROS 1 上位机程序的工程。树莓派负责感知、建图、导航与目标跟踪；STM32 负责编码器采样、车辆运动学、速度闭环和电机 PWM。两端通过 USB 串口连接，ROS 速度话题最终会转换为 STM32 可识别的二进制帧。

> 本仓库中的 ROS 包来自随车资料，面向 ROS 1 与 catkin 工作空间。不要将当前 `main` 分支当作 ROS 2 工程使用。

## 1. 系统结构

```text
相机 / 激光雷达 / 上位机算法
              |
              v
树莓派 ROS 1
  - 建图、定位、导航、视觉跟踪
  - 发布 /cmd_vel
  - 发布 /odom、/imu、/PowerVoltage
              |
              | /dev/wheeltec_controller, 115200 bit/s
              v
STM32F103VET6
  - 串口协议解析
  - 车型运动学与四路速度 PI
  - 编码器、IMU、电压采样
  - TIM8 PWM 与电机方向控制
              |
              v
电机、霍尔编码器、IMU 与电池
```

### 1.1 职责边界

| 层级 | 主要职责 | 核心代码 |
| --- | --- | --- |
| 树莓派 ROS | 感知、建图、导航、视觉跟踪和速度决策 | `src/turn_on_wheeltec_robot`、`src/kcf_track` |
| 串口桥接 | `/cmd_vel` 与 STM32 二进制协议互转，发布里程计与 IMU | `src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp` |
| STM32 | 实时控制、编码器闭环、PWM 输出和基础保护 | `F103VET6_Mini小车_STM32源码_2022.01.06(霍尔编码器)` |

## 2. 仓库结构

```text
mini_car/
├── F103VET6_Mini小车_STM32源码_2022.01.06(霍尔编码器)/
│   ├── USER/                         Keil 工程入口与中断配置
│   ├── BALANCE/                      运动学、速度 PI、车型参数、MPU9250
│   ├── HARDWARE/                     电机、编码器、串口、CAN、ADC 等驱动
│   ├── FreeRTOS/                     FreeRTOS 内核与 ARM CM3 移植层
│   └── USER/WHEELTEC.uvprojx         Keil MDK 工程文件
├── src/
│   ├── turn_on_wheeltec_robot/       ROS 底盘通信、TF、导航与建图配置
│   └── kcf_track/                    OpenCV KCF 视觉目标跟踪
├── .gitignore                        构建产物与本机配置忽略规则
├── .gitattributes                    跨 Windows / Linux 的行尾规则
└── README.md                         本文档
```

## 3. STM32 底盘固件

### 3.1 构建环境

- 芯片：STM32F103VET6。
- 工程：`USER/WHEELTEC.uvprojx`。
- IDE：Keil MDK，历史构建使用 ARMCC 5.06。
- 时钟：72 MHz。
- RTOS：FreeRTOS 9.0.0，系统 Tick 为 1 ms。
- 编码器版本：本目录明确适配霍尔编码器。

使用 Keil 打开 [WHEELTEC.uvprojx](F103VET6_Mini小车_STM32源码_2022.01.06(霍尔编码器)/USER/WHEELTEC.uvprojx) 后，选择目标 `FreeRTOS` 并编译。构建输出在 `OBJ/`，已由 Git 忽略。

### 3.2 任务与控制周期

| FreeRTOS 任务 | 优先级 | 周期 | 工作内容 |
| --- | ---: | ---:| --- |
| `Balance_task` | 4 | 10 ms | 读取编码器、处理控制源、逆运动学、PI 和 PWM |
| `MPU9250_task` | 3 | 10 ms | 读取加速度、陀螺仪、磁力计 |
| `pstwo_task` | 4 | 10 ms | 采集 PS2 手柄 |
| `data_task` | 4 | 50 ms | 向串口和 CAN 上报底盘状态 |
| `show_task` | 3 | 100 ms | 读取电压、OLED 和蜂鸣器 |
| `led_task` | 3 | 动态 | 状态灯闪烁 |

主控制路径位于 `BALANCE/BALANCE/balance.c`：以 100 Hz 读取四路编码器，根据车型将 `Vx`、`Vy`、`Vz` 分解为轮速目标，并通过四个增量式 PI 控制器驱动 TIM8 的四路 PWM。

### 3.3 车型选择

开机时，STM32 通过 ADC8 读取电位器档位，选择车型参数。当前源码支持：

| 枚举值 | 车型 | 特性 |
| ---: | --- | --- |
| `0` | `Mec_Car` | 四轮麦克纳姆，可横移 |
| `1` | `Omni_Car` | 三轮全向，可横移 |
| `2` | `Akm_Car` | 阿克曼，带舵机 |
| `3` | `Diff_Car` | 两轮差速 |
| `4` | `FourWheel_Car` | 四驱 |
| `5` | `Tank_Car` | 履带 / 差速式 |

ROS 启动文件 `turn_on_wheeltec_robot.launch` 的默认模型为 `mini_mec`。实车车型必须与 STM32 电位器档位及 ROS 传入的 `car_mode` 对应，否则运动学、TF 外形和导航参数会不一致。

### 3.4 STM32 侧安全条件

电机输出需同时满足以下条件：

- 电池电压不低于 10 V。
- 硬件使能开关有效。
- MPU9250 启动零偏校准完成，软件停止标志已释放。

当前底盘固件不具备树莓派通信超时自动停车机制。不要在实际运动中依赖“拔掉串口或断开 ROS”来停止小车；应发送零速度、关闭使能开关，或优先增加通信看门狗后再进行无人值守测试。

## 4. 树莓派 ROS 1 部分

### 4.1 `turn_on_wheeltec_robot`

该包是 ROS 与 STM32 底盘之间的桥接层，编译目标为 `wheeltec_robot_node`。

它会：

1. 订阅 `cmd_vel`，接收 `geometry_msgs/Twist`。
2. 把 `linear.x`、`linear.y`、`angular.z` 编码为 STM32 下行速度帧。
3. 从串口读取 STM32 的状态帧。
4. 发布 `odom`、`imu` 和 `PowerVoltage`。
5. 发布 `odom`、`base_footprint`、相机等 TF 关系所需的配置。

核心文件：

- [wheeltec_robot.cpp](src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp)：串口收发、里程计积分和 ROS 话题发布。
- [base_serial.launch](src/turn_on_wheeltec_robot/launch/include/base_serial.launch)：底盘设备名、波特率和速度平滑入口。
- [turn_on_wheeltec_robot.launch](src/turn_on_wheeltec_robot/launch/turn_on_wheeltec_robot.launch)：底盘、机器人模型和导航组件总启动文件。

### 4.2 `kcf_track`

该包使用 OpenCV 的 KCF 目标跟踪器订阅 RGB 与深度图像，输出目标距离和像素角度；`kcf_follow.py` 再计算跟随速度并发布 `cmd_vel`。启动文件为：

```bash
roslaunch kcf_track kcf_tracker.launch
```

该启动文件会同时启动底盘节点和 Astra 相机。运行 KCF 跟踪前，请确保现场没有另一个已在运行的底盘节点或相机节点，以避免同名节点、串口和相机设备竞争。

### 4.3 ROS 话题

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS -> STM32 | 目标速度。使用 `linear.x`、`linear.y`、`angular.z`。 |
| `/odom` | `nav_msgs/Odometry` | STM32 -> ROS | 基于编码器积分得到的里程计。 |
| `/imu` | `sensor_msgs/Imu` | STM32 -> ROS | STM32 回传的 MPU9250 原始数据换算结果。 |
| `/PowerVoltage` | `std_msgs/Float32` | STM32 -> ROS | 电池电压，单位 V。 |
| `/scan` | `sensor_msgs/LaserScan` | 雷达 -> ROS | 2D 建图和导航的激光雷达数据。 |
| `/camera/rgb/image_raw` | `sensor_msgs/Image` | 相机 -> ROS | RGB 图像。 |
| `/camera/depth/image` | `sensor_msgs/Image` | 相机 -> ROS | 深度图像。 |
| `/camera/rgb/camera_info` | `sensor_msgs/CameraInfo` | 相机 -> ROS | RGB 相机内参。 |
| `/rtabmap/grid_map` | `nav_msgs/OccupancyGrid` | RTAB-Map -> ROS | RTAB-Map 生成的栅格地图。 |

## 5. 树莓派与 STM32 串口通信

### 5.1 连接方式

底盘节点默认打开 `/dev/wheeltec_controller`，波特率为 `115200`。该软链接由 `scripts/wheeltec_udev.sh` 创建，用于避免重启后 USB 串口序号变化。

在连接控制板前，先检查：

```bash
ls -l /dev/wheeltec_controller
lsusb
udevadm info -a -n /dev/ttyUSB0
```

`wheeltec_udev.sh` 中的 VID、PID 和序列号是随车资料中的示例。务必使用实际设备信息核对后再执行规则脚本，避免将雷达错误绑定为控制器。

若仅临时调试，可修改 `base_serial.launch` 的 `usart_port_name` 为实际设备，如 `/dev/ttyUSB0`。长期部署应使用稳定的 udev 软链接。

### 5.2 下行控制帧：ROS 到 STM32

总长 11 字节，所有速度均为有符号 16 位大端整数，数值是 ROS 单位乘以 1000。

| 字节索引 | 内容 | 说明 |
| ---: | --- | --- |
| 0 | `0x7B` | 帧头 |
| 1-2 | 保留 | 固定为 0 |
| 3-4 | `Vx * 1000` | `linear.x`，m/s 转 mm/s |
| 5-6 | `Vy * 1000` | `linear.y`，m/s 转 mm/s |
| 7-8 | `Vz * 1000` | `angular.z`，rad/s 放大 1000 倍 |
| 9 | XOR | 字节 0 至 8 的异或校验 |
| 10 | `0x7D` | 帧尾 |

### 5.3 上行状态帧：STM32 到 ROS

总长 24 字节：帧头、停止状态、三轴底盘速度、三轴加速度、三轴角速度、电池电压、异或校验和帧尾。速度和电压均采用放大 1000 倍的有符号 16 位整数；IMU 原始量由 ROS 节点按当前 MPU9250 量程换算。

## 6. 安装与构建

### 6.1 系统要求

建议使用 Raspberry Pi OS / Ubuntu 上的 ROS 1 环境。源码兼容 catkin，实际所需依赖还取决于要启用的功能：

- 基础底盘：`roscpp`、`serial`、`tf`、`nav_msgs`、`sensor_msgs`、`geometry_msgs`。
- 视觉跟踪：OpenCV、`cv_bridge`、`image_transport`、Astra 相机驱动。
- 2D 建图与导航：雷达驱动、`gmapping` 或其他 SLAM 包、`move_base`、`amcl`、地图服务。
- 3D 建图与导航：`rtabmap_ros`、Astra 相机驱动、雷达驱动、导航栈。

查看当前 ROS 发行版：

```bash
rosversion -d
```

对于 Noetic，可优先安装二进制 RTAB-Map 包：

```bash
sudo apt update
sudo apt install ros-noetic-rtabmap-ros
```

若使用 Melodic，请将上面的 `noetic` 替换为 `melodic`，并确认该发行版的软件源提供对应包。没有二进制包时，再根据 [RTAB-Map 官方安装文档](https://github.com/introlab/rtabmap/wiki/Installation) 从源码构建。官方当前默认分支是 ROS 2；本项目使用 ROS 1，因此不要直接把 ROS 2 分支加入本工作空间。

### 6.2 放置仓库

以下示例假定 catkin 工作空间是 `~/catkin_ws`：

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://gitee.com/qbz23/mini_car_project.git mini_car
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

为使每次新终端自动加载工作空间，可在确认路径无误后加入 `~/.bashrc`：

```bash
source ~/catkin_ws/devel/setup.bash
```

### 6.3 基础自检

先只启动底盘和模型：

```bash
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch
```

另开一个终端检查：

```bash
rostopic list
rostopic echo -n 1 /PowerVoltage
rostopic echo -n 1 /odom
rosrun tf tf_echo odom_combined base_footprint
```

在悬空状态下再测试零速度，随后才进行低速直线测试：

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

确认方向、轮速与里程计正确后，才可逐步提高速度。实车测试应始终确保急停、使能开关和机械支撑可用。

## 7. 常用启动方式

### 7.1 2D 建图

```bash
roslaunch turn_on_wheeltec_robot mapping.launch
```

`mapping.launch` 默认使用 `gmapping`，并启动雷达、底盘节点和机器人模型。可通过 `mapping_mode` 切换为 `hector` 或 `karto`，前提是相应 ROS 包已经安装。

### 7.2 已有地图的 2D 导航

```bash
roslaunch turn_on_wheeltec_robot navigation.launch
```

默认地图为 `src/turn_on_wheeltec_robot/map/WHEELTEC.yaml`。替换地图时，应同时替换配套 `.yaml` 和 `.pgm` 文件，或通过 `map_file` 参数传入新地图路径。

### 7.3 RTAB-Map 3D 建图

```bash
roslaunch turn_on_wheeltec_robot 3d_mapping.launch
```

该启动文件会启动 2D 建图、Astra 相机和 RTAB-Map。为适配树莓派负载，当前设置会将深度分辨率降为 `320x240`。RTAB-Map 使用 `/odom`、`/scan`、RGB 图和深度图；启动失败时优先检查这些话题与相机 TF。

### 7.4 RTAB-Map 3D 导航

```bash
roslaunch turn_on_wheeltec_robot 3d_navigation.launch
```

该流程将 `/rtabmap/grid_map` 作为 AMCL 与 `move_base` 的地图输入。树莓派性能有限时，应关闭 `rtabmapviz`、降低图像分辨率、减慢车速并减少不必要的同时运行节点。

### 7.5 KCF 视觉跟随

```bash
roslaunch kcf_track kcf_tracker.launch
```

KCF 相关参数位于 `kcf_tracker.launch`：最大线速度为 `0.3 m/s`，最大角速度为 `0.4 rad/s`。应从保守速度开始调参，避免目标丢失或深度无效时出现不期望运动。

## 8. RTAB-Map 适配说明

本项目已经包含 `rtabmap_ros` 的启动配置，但不包含 RTAB-Map 本体源码。安装完成后，至少满足以下条件才能正常使用：

| 条件 | 验证命令 |
| --- | --- |
| RTAB-Map 已安装 | `roscd rtabmap_ros` |
| RGB 图像可用 | `rostopic hz /camera/rgb/image_raw` |
| 深度图可用 | `rostopic hz /camera/depth/image` |
| 相机内参可用 | `rostopic echo -n 1 /camera/rgb/camera_info` |
| 轮速里程计可用 | `rostopic echo -n 1 /odom` |
| 雷达数据可用 | `rostopic echo -n 1 /scan` |
| TF 链完整 | `rosrun tf view_frames` |

对 Raspberry Pi 3/4 而言，RTAB-Map 的计算量较高。优先使用二进制包，保持低分辨率与低速移动；如果从源码编译，需预留足够内存和交换空间。有关依赖、源码构建与树莓派限制，请以 [官方安装说明](https://github.com/introlab/rtabmap/wiki/Installation) 为准。

## 9. 常见问题

### 9.1 找不到 `/dev/wheeltec_controller`

检查 USB 连接和设备枚举：

```bash
dmesg | tail -n 50
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

若有实际串口但没有软链接，核对 udev 规则的 VID、PID 和序列号。不要盲目执行或复制随车规则脚本。

### 9.2 底盘节点能启动但没有 `/odom`

依次检查串口设备、115200 波特率、STM32 固件是否已烧录、底盘使能开关、电池电压，以及控制板是否确实连接到 STM32 的 USART3 通信口。

### 9.3 发送 `/cmd_vel` 后不动

确认 STM32 完成约 10 秒的 IMU 零偏校准，电压高于保护阈值，使能开关打开，且 ROS 速度话题由 `wheeltec_robot_node` 订阅：

```bash
rostopic info /cmd_vel
```

### 9.4 RTAB-Map 无法生成地图

确认 `rtabmap_ros` 已安装，并按本 README 的 3D 自检表检查 RGB、深度、雷达、里程计和 TF。相机深度与 RGB 时间戳不同步时，`rgbd_sync` 会导致数据不进入 RTAB-Map。

### 9.5 `catkin_make` 找不到包或依赖

先确认仓库放置在工作空间的 `src/` 内，再执行：

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

外设驱动和导航包属于系统依赖，未必包含在本仓库中，应按所使用的相机、雷达和 ROS 发行版单独安装。

## 10. Git 版本管理

### 10.1 分支约定

- `main`：始终保存可构建、可部署、经过基础验证的版本。
- `feature/<功能>`：新增功能，例如 `feature/serial-watchdog`。
- `fix/<问题>`：缺陷修复，例如 `fix/odom-frame`。
- `docs/<主题>`：仅文档修改，例如 `docs/deployment-guide`。

### 10.2 日常开发流程

```bash
git switch main
git pull --ff-only
git switch -c feature/serial-watchdog

# 修改代码，并在树莓派完成构建和实车验证
git diff --check
git status
git add src/turn_on_wheeltec_robot
git commit -m "feat: 增加串口通信看门狗"

git switch main
git merge --no-ff feature/serial-watchdog
git push origin main
```

提交信息建议使用中文或 Conventional Commits 形式，保持简短并描述结果：

```text
feat: 增加串口通信看门狗
fix: 修正阿克曼车型的 TF 参数
docs: 补充树莓派部署步骤
```

### 10.3 发布版本

每次实车验证通过后可创建标签：

```bash
git tag -a v0.1.0 -m "树莓派 ROS 底盘通信整合版"
git push origin v0.1.0
```

### 10.4 Git 忽略规则

`.gitignore` 已排除 catkin 的 `build/`、`devel/`、`install/`、`log/`，Python 缓存，Keil 的 `OBJ/` 和其他生成文件。不要提交编译产物或设备上的日志；应提交源码、启动文件、参数、接线说明和必要的地图源文件。

## 11. 开发与安全建议

1. 修改底盘协议、车型运动学或电机参数前，先创建 Git 分支并保留可回退标签。
2. 首次测试必须让驱动轮悬空，确认轮子方向与编码器符号正确后再落地。
3. 优先在 ROS 层加入 `/cmd_vel` 超时置零和速度限制，并在 STM32 层加入独立通信看门狗。
4. 不要同时运行多个会启动底盘串口、相机或雷达的 launch 文件。
5. 使用 `rosbag record` 记录 `/odom`、`/imu`、`/scan`、相机数据和 `/cmd_vel`，便于复现导航与定位问题。
6. `.gitattributes` 规定源码以 LF 进入版本库；Windows 编辑后出现行尾提示时，先检查实际差异，不要把全量换行变更混入功能提交。

## 12. 参考资料

- [RTAB-Map 安装文档](https://github.com/introlab/rtabmap/wiki/Installation)
- [RTAB-Map ROS 仓库](https://github.com/introlab/rtabmap_ros)
- [项目 Gitee 仓库](https://gitee.com/qbz23/mini_car_project)
