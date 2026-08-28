# Mini ROS2 小车工程（厂房救援）

面向厂房救援场景的轮式机器人工程：STM32F103VET6 底盘固件 + ROS2 串口桥接 + RTAB-Map 实时建图 + Nav2 导航 + KCF 视觉跟随，并附带一套不依赖 ROS 的 Web 救援控制台，可通过浏览器实时查看建图、遥操小车与下发导航目标。

核心价值：**救援任务无法提前建图**，因此本工程以 RTAB-Map 增量记忆模式边行驶边建图、边定位边导航，并通过 Web 控制台把实时地图、实时画面与遥控能力交给操作员。

智能体或新开发者接手前，请先阅读 [智能体交接文档](docs/AGENT_HANDOFF.md)、[仓库工作约束](AGENTS.md) 和 [第三方组件说明](docs/THIRD_PARTY_NOTICES.md)。

## 1. 核心功能特性

| 能力 | 说明 |
| --- | --- |
| 实时 SLAM 建图 | RTAB-Map 增量记忆模式，RGB-D + 2D 激光融合，边走边建，发布 `/map` 与 `map -> odom` |
| 实时导航 | Nav2 全局/局部规划，点击目标点即自动行驶；网关走 `NavigateToPose` action 并回传到达/失败/取消 |
| Web 救援控制台 | FastAPI 网关 + 浏览器页面：实时地图、激光扫描、行驶轨迹、手动遥操、MJPEG 实时画面 |
| 彩色实时画面 | `/camera/color/image_raw` 转 MJPEG 推流（限流 10fps），供救援遥操看路 |
| 多车型底盘 | 麦克纳姆、全向三轮、阿克曼、两轮差速、四轮驱动，STM32 与 ROS2 `model` 参数对应 |
| 视觉跟随 | KCF RGB-D 目标跟踪，支持 ROI 指定与跟踪超时停车 |

## 2. 运行前置条件

### 2.1 目标机（运行 ROS2 与小车）

- Ubuntu 22.04（aarch64 或 x86-64）
- ROS2 Humble
- Raspberry Pi 4 / Raspberry Pi 5（或同等算力主机）
- Python ≥ 3.10（Web 控制台使用）
- colcon + ament_cmake

### 2.2 硬件

| 设备 | 稳定设备路径 | 说明 |
| --- | --- | --- |
| STM32 控制板 | `/dev/wheeltec_controller` | CP2102 序列号 0002，115200 bit/s，8N1，对应 STM32 USART3 |
| RPLIDAR A1M8 | `/dev/wheeltec_lidar` | CP2102 序列号 0001，115200，默认 Standard 模式 |
| Orbbec Astra Pro | USB `2bc5:0403`（深度）/ `2bc5:0502`（彩色 UVC） | 深度与彩色为两个独立 USB 设备 |

### 2.3 系统依赖

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-rtabmap-ros \
  ros-humble-cv-bridge \
  ros-humble-robot-state-publisher \
  ros-humble-image-transport \
  ros-humble-image-publisher \
  ros-humble-image-geometry \
  ros-humble-camera-info-manager \
  ros-humble-tf2-eigen \
  ros-humble-tf2-sensor-msgs \
  build-essential cmake git \
  libgflags-dev libgoogle-glog-dev nlohmann-json3-dev \
  libusb-1.0-0-dev
```

相机驱动的两个第三方库需源码安装（低内存设备保持单线程）：

```bash
mkdir -p ~/camera_dependencies && cd ~/camera_dependencies

git clone --depth 1 --branch v0.8.0 https://github.com/Neargye/magic_enum.git
cmake -S magic_enum -B magic_enum/build
cmake --build magic_enum/build --parallel 1
sudo cmake --install magic_enum/build

git clone --depth 1 https://github.com/libuvc/libuvc.git
cmake -S libuvc -B libuvc/build -DBUILD_EXAMPLES=OFF
cmake --build libuvc/build --parallel 1
sudo cmake --install libuvc/build
sudo ldconfig
```

## 3. 主要目录结构

```text
mini_car/
├── rescue_console/              Web 救援控制台（不依赖 ROS 的客户端 + 小车侧网关）
│   ├── server/
│   │   ├── app.py               FastAPI 网关：REST + WebSocket + MJPEG 流
│   │   ├── bridge.py            桥接层：BaseBridge 接口 + RosBridge（rclpy）
│   │   └── requirements.txt     Web 依赖（fastapi / uvicorn / websockets / Pillow）
│   ├── web/index.html           控制页：实时地图、激光、轨迹、遥操、实时画面
│   └── deploy/
│       ├── install.sh           可选：注册 systemd 开机自启
│       └── rescue-console.service.template
├── src/
│   ├── turn_on_wheeltec_robot/  ROS2 底盘串口桥接、URDF、RTAB-Map/Nav2 启动配置
│   │   ├── config/              wheeltec_bridge.yaml（串口参数）、nav2_params.yaml
│   │   ├── launch/              ROS2 入口（*.launch.py）
│   │   ├── src/                 wheeltec_robot.cpp 串口桥接节点
│   │   ├── urdf/                车型模型
│   │   ├── map/                 示例地图（WHEELTEC.pgm / .yaml）
│   │   └── scripts/             wheeltec_udev.sh、send_mark.py
│   ├── astra_camera/            Astra Pro 驱动与随车 OpenNI2 二进制库
│   ├── astra_camera_msgs/       Astra Pro 自定义消息与服务
│   ├── rplidar_ros/             SLAMTEC RPLIDAR ROS2 驱动
│   └── kcf_track/               KCF 目标跟踪（launch / scripts / src）
├── F103VET6_Mini小车_STM32源码_2022.01.06(霍尔编码器)/
│   ├── USER/                    Keil 工程与程序入口
│   ├── BALANCE/                 运动学、速度 PI、车型参数
│   ├── HARDWARE/                电机、编码器、串口、CAN、ADC
│   └── FreeRTOS/                FreeRTOS 9
├── docs/                        AGENT_HANDOFF.md、THIRD_PARTY_NOTICES.md
├── AGENTS.md                    仓库工作约束
└── README.md
```

### 3.1 ROS2 launch 入口

`src/turn_on_wheeltec_robot/launch/` 下的 ROS2 入口：

| 文件 | 用途 |
| --- | --- |
| `slam_navigation.launch.py` | **主入口**：实时 SLAM + Nav2 导航（默认自动启动底盘、雷达、相机） |
| `base.launch.py` | 仅底盘串口桥接 + URDF + 静态 TF |
| `rtabmap_mapping.launch.py` | 纯建图（生成数据库，不启动 Nav2） |
| `rtabmap_navigation.launch.py` | 基于已有数据库的定位导航 |
| `astra_pro.launch.py` | 单独启动 Astra Pro 相机 |
| `rplidar_a1.launch.py` | 单独启动 RPLIDAR A1M8 |

> 实物只有 Orbbec Astra Pro（深度 `2bc5:0403`、彩色 UVC `2bc5:0502`）。同目录下的 `astra_s.launch.py` 与 `src/astra_camera/launch/` 内的 Astra Mini / Pro Plus 等入口对应仓库里没有的型号，**不适用本设备**，不要照抄使用。

同目录下的 `*.launch`（如 `mapping.launch`、`navigation.launch`、`turn_on_wheeltec_robot.launch`）是旧 ROS1 XML launch，仅作迁移对照保留；`CMakeLists.txt` 只安装 `*.launch.py`，它们不会进入 ROS2 安装空间。`send_mark.py`、`imageResize.py` 同理不参与 ROS2 安装。

## 4. 安装与构建

**仓库根目录即 colcon 工作空间**（`~/mini_car_ws/src` 为源码目录），因此直接把仓库克隆到 `~/mini_car_ws`：

```bash
git clone git@gitee.com:qbz23/mini_car_project.git ~/mini_car_ws
cd ~/mini_car_ws

source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

低内存树莓派建议按包分步、单线程构建，避免 OOM：

```bash
colcon build --packages-select astra_camera_msgs astra_camera \
  --symlink-install --parallel-workers 1
colcon build --packages-select rplidar_ros \
  --symlink-install --parallel-workers 1
colcon build --packages-select turn_on_wheeltec_robot kcf_track \
  --symlink-install --parallel-workers 1
```

建议写入 `~/.bashrc`：

```bash
source /opt/ros/humble/setup.bash
source ~/mini_car_ws/install/setup.bash
```

## 5. 基础用法

> **实车安全第一**：首次电机测试必须让车轮悬空，并确认硬件使能开关随手可关。

### 5.1 验证底盘

```bash
ros2 launch turn_on_wheeltec_robot base.launch.py \
  model:=mini_mec \
  serial_port:=/dev/wheeltec_controller
```

另开终端检查：

```bash
ros2 topic echo --once /PowerVoltage
ros2 topic echo --once /odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

**确认车轮悬空后**低速点动：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

停止发布后，桥接节点会在默认 0.5 秒内发送零速度。

### 5.2 实时 SLAM 建图 + 导航（主入口）

```bash
ros2 launch turn_on_wheeltec_robot slam_navigation.launch.py \
  model:=mini_mec \
  serial_port:=/dev/wheeltec_controller \
  database_path:=$HOME/.ros/mini_car_slam.db
```

默认自动启动 Astra Pro 与 A1M8。若已在其他终端启动，需禁用重复启动：

```bash
ros2 launch turn_on_wheeltec_robot slam_navigation.launch.py start_astra_camera:=false
ros2 launch turn_on_wheeltec_robot slam_navigation.launch.py start_lidar:=false
```

实时 SLAM 的最小数据闭环：

- `/camera/color/image_raw`、`/camera/depth/image_raw`、`/camera/color/camera_info`
- `/scan`
- `/odom`
- TF：`map -> odom -> base_footprint -> camera_link`

实时导航时**不要**启动 `map_server` 加载旧地图，也**不要**启动 AMCL 作为第二个定位源——地图由 RTAB-Map 在线发布，Nav2 全局代价地图订阅 `/map`。

### 5.3 Web 救援控制台

在小车侧启动网关（浏览器客户端不依赖任何 ROS 库）：

```bash
cd ~/mini_car_ws
git pull --ff-only

python3 -m venv rescue_console/venv
./rescue_console/venv/bin/pip install -r rescue_console/server/requirements.txt

source /opt/ros/humble/setup.bash
source ~/mini_car_ws/install/setup.bash
cd ~/mini_car_ws/rescue_console/server
./../../venv/bin/python -m uvicorn app:app --host 0.0.0.0 --port 8000
```

浏览器访问 `http://<目标机IP>:8000`。

页面操作：

| 功能 | 操作 |
| --- | --- |
| 实时地图 | 自动渲染占用栅格、激光扫描与行驶轨迹 |
| 手动遥操 | `W/S/A/D/Q/E` 或按住屏幕按钮，10Hz 心跳 |
| 紧急停车 | 「停」按钮，取消导航并立即发零速度 |
| 导航目标 | 点击地图任意位置，由 Nav2 规划并行驶 |
| 实时画面 | 自动播放 MJPEG 彩色流 |

详细协议、开机自启与排障见 [rescue_console/README.md](rescue_console/README.md)。

需要开机自启时：

```bash
cd ~/mini_car_ws/rescue_console/deploy
./install.sh
systemctl status rescue-console
journalctl -u rescue-console -f
```

### 5.4 纯建图（生成数据库）

```bash
ros2 launch turn_on_wheeltec_robot rtabmap_mapping.launch.py \
  model:=mini_mec \
  database_path:=$HOME/.ros/mini_car_rtabmap.db
```

该 launch 使用 `-d`，**每次启动都会删除同路径旧数据库**。建图完成后及时备份：

```bash
cp ~/.ros/mini_car_rtabmap.db \
  ~/.ros/mini_car_rtabmap_$(date +%Y%m%d_%H%M%S).db
```

### 5.5 基于数据库的定位导航

```bash
test -f ~/.ros/mini_car_rtabmap.db && echo "数据库存在"

ros2 launch turn_on_wheeltec_robot rtabmap_navigation.launch.py \
  model:=mini_mec \
  database_path:=$HOME/.ros/mini_car_rtabmap.db
```

与实时 SLAM 入口的区别：`rtabmap_navigation.launch.py` 使用定位模式，只用于加载已保存数据库后的导航；`slam_navigation.launch.py` 使用增量记忆模式边建图边导航。

### 5.6 KCF 视觉跟随

```bash
ros2 launch kcf_track kcf_tracking.launch.py \
  model:=mini_mec \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw
```

桌面环境下在 KCF RGB 窗口拖动鼠标选择目标；无显示器时指定初始 ROI：

```bash
ros2 run kcf_track kcf_node --ros-args \
  -p show_window:=false \
  -p initial_roi.x:=200 \
  -p initial_roi.y:=120 \
  -p initial_roi.width:=120 \
  -p initial_roi.height:=160
```

KCF 与 Nav2 都会发布 `/cmd_vel`，**不能直接同时控制底盘**。并行运行需部署 `twist_mux` 并制定优先级。

## 6. 系统架构

```text
RGB-D 相机 --------> RTAB-Map / KCF
                         |
激光雷达 ----------> RTAB-Map / Nav2
                         |
                         v
                map -> odom -> base_footprint
                         |
Nav2 / KCF ----------> /cmd_vel
                         |
                         v
                wheeltec_robot_node
                         |
       /dev/wheeltec_controller, 115200 bit/s
                         |
                         v
                    STM32 USART3
                         |
                         v
               编码器闭环、PWM、电机

浏览器 Web 控制台 <--HTTP/WebSocket/MJPEG--> FastAPI 网关 <--rclpy--> ROS2 话题
```

| 模块 | 职责 |
| --- | --- |
| RTAB-Map | RGB-D + 激光雷达建图、回环检测、定位，发布 `map -> odom` |
| Nav2 | 全局规划、局部避障、行为树导航，发布 `/cmd_vel` |
| KCF | 目标框跟踪、目标深度估计和跟随速度计算 |
| ROS2 底盘桥接 | ROS2 话题与 STM32 二进制协议互转，发布 `odom -> base_footprint` |
| Web 救援控制台 | 把 ROS2 话题封装为 HTTP/WebSocket/MJPEG，客户端零 ROS 依赖 |
| STM32 | 车型运动学、四路轮速 PI、编码器与 IMU 采集、电机 PWM |

## 7. ROS2 迁移说明

底盘桥接已从 roscpp/catkin 迁移为 rclcpp/ament：

| ROS1 | ROS2 |
| --- | --- |
| roscpp | rclcpp |
| catkin | ament_cmake |
| catkin_make | colcon build |
| tf | tf2_ros |
| serial ROS 包 | Linux termios |
| XML .launch | Python .launch.py |

新底盘节点增加了：

- 串口断开后每 2 秒自动重连。
- 流式 24 字节状态帧解析和错位重同步。
- `/cmd_vel` 超时后主动向 STM32 发送零速度。
- 速度限幅和 int16 协议溢出保护。
- `/chassis_enabled` 状态话题。
- 平面里程计积分和 `odom -> base_footprint` TF。

KCF 节点已迁移为 rclcpp，跟随节点已迁移为 Python 3 + rclpy，支持 32FC1/16UC1 深度图、ROI 边界检查和跟踪超时停车。

## 8. STM32 固件

### 8.1 基本信息

- MCU：STM32F103VET6
- 主频：72 MHz
- RTOS：FreeRTOS 9.0.0
- 控制周期：100 Hz
- PWM：TIM8 四通道，约 10 kHz
- 默认速度 PI：Kp=300、Ki=300

Keil 工程：

```text
F103VET6_Mini小车_STM32源码_2022.01.06(霍尔编码器)/USER/WHEELTEC.uvprojx
```

### 8.2 车型对应

| STM32 枚举 | ROS2 model | 车型 |
| --- | --- | --- |
| Mec_Car | mini_mec | 麦克纳姆 |
| Omni_Car | mini_omni | 三轮全向 |
| Akm_Car | mini_akm | 阿克曼 |
| Diff_Car | mini_diff | 两轮差速 |
| FourWheel_Car | mini_4wd | 四轮驱动 |

STM32 电位器档位必须与 ROS2 的 `model` 参数一致。

### 8.3 安全提示

STM32 原固件没有通信超时停车机制。ROS2 桥接已增加上位机看门狗，但**仍建议在 STM32 侧增加独立通信看门狗**——树莓派死机时 ROS2 无法继续发送零速度。

## 9. 串口协议

> STM32 通信协议保持不变，因此已烧录的霍尔编码器版固件可以继续使用。

默认设备 `/dev/wheeltec_controller`，115200 bit/s，8N1，对应 STM32 USART3。

### 9.1 ROS2 到 STM32

总长 11 字节：

| 索引 | 数据 |
| ---: | --- |
| 0 | 帧头 0x7B |
| 1-2 | 保留，固定为 0 |
| 3-4 | linear.x * 1000，有符号大端 int16 |
| 5-6 | linear.y * 1000，有符号大端 int16 |
| 7-8 | angular.z * 1000，有符号大端 int16 |
| 9 | 字节 0 至 8 的 XOR |
| 10 | 帧尾 0x7D |

### 9.2 STM32 到 ROS2

总长 24 字节：

| 索引 | 数据 |
| ---: | --- |
| 0 | 帧头 0x7B |
| 1 | STM32 软件停止标志 |
| 2-7 | Vx、Vy、Vz |
| 8-13 | 三轴加速度原始值 |
| 14-19 | 三轴角速度原始值 |
| 20-21 | 电池电压，放大 1000 倍 |
| 22 | 字节 0 至 21 的 XOR |
| 23 | 帧尾 0x7D |

## 10. ROS2 话题与 TF

| 名称 | 类型 | 方向 |
| --- | --- | --- |
| `/cmd_vel` | geometry_msgs/msg/Twist | Nav2/KCF -> 底盘 |
| `/odom` | nav_msgs/msg/Odometry | 底盘 -> ROS2 |
| `/imu` | sensor_msgs/msg/Imu | 底盘 -> ROS2 |
| `/PowerVoltage` | std_msgs/msg/Float32 | 底盘 -> ROS2 |
| `/chassis_enabled` | std_msgs/msg/Bool | 底盘 -> ROS2 |
| `/scan` | sensor_msgs/msg/LaserScan | 雷达 -> RTAB-Map/Nav2 |
| `/camera/color/image_raw` | sensor_msgs/msg/Image | 相机 -> RTAB-Map/KCF/Web 网关 |
| `/camera/depth/image_raw` | sensor_msgs/msg/Image | 相机 -> RTAB-Map/KCF |
| `/camera/color/camera_info` | sensor_msgs/msg/CameraInfo | 相机 -> RTAB-Map |
| `/map` | nav_msgs/msg/OccupancyGrid | RTAB-Map -> Nav2/Web 网关 |

TF 链：

```text
map                 RTAB-Map 发布
└── odom            底盘里程计参考系
    └── base_footprint
        ├── base_link
        ├── imu_link
        └── camera_link
```

RTAB-Map 发布 `map -> odom`，底盘节点发布 `odom -> base_footprint`。**不要启动第二个发布相同 TF 的节点。**

### 10.1 深度相机在建图中的角色

`slam_navigation.launch.py` 中 RTAB-Map 同时订阅 RGB-D 与激光（`subscribe_rgbd: true`、`subscribe_scan: true`）：

- **RGB-D**（Astra Pro）负责视觉特征、回环检测、图优化约束与 3D 重建；
- **2D 激光**（A1M8）负责生成 2D 占用栅格 `/map`（`Grid/FromDepth: false`），供 Nav2 做 2D 代价地图与规划。

两者都在参与 SLAM，只是分工不同。厂房为平面环境、Nav2 为 2D 导航，因此 2D 栅格由激光生成，比深度图投影更稳定。

## 11. Astra Pro 深度相机驱动

Astra Pro 驱动已作为随车资料源码纳入仓库，目标机拉取仓库后应已有：

```text
~/mini_car_ws/src/astra_camera
~/mini_car_ws/src/astra_camera_msgs
~/mini_car_ws/src/rplidar_ros
```

第三方来源与许可边界见 [docs/THIRD_PARTY_NOTICES.md](docs/THIRD_PARTY_NOTICES.md)。

**不要在工作空间内额外解压第二份 Astra 驱动**，否则 colcon 会报 Duplicate package names。

单独验证相机（`slam_navigation.launch.py` 会开启 UVC 彩色流、深度对齐与 RGB/Depth 同步）：

```bash
ros2 launch turn_on_wheeltec_robot astra_pro.launch.py

ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo --once /camera/color/camera_info
```

若相机已在其他终端启动，运行 SLAM 时必须禁止重复启动（见 5.2）。

## 12. udev 串口配置

先确定实际设备：

```bash
lsusb
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
udevadm info -a -n /dev/ttyUSB0
```

随车脚本：

```text
src/turn_on_wheeltec_robot/scripts/wheeltec_udev.sh
```

脚本中的 VID、PID 和序列号是厂家示例，执行前**必须与实物核对**。

> 不要把 `ttyUSB0`/`ttyUSB1` 当作永久设备身份——插拔顺序变化会导致串号漂移，必须使用稳定的 udev 软链接（`/dev/wheeltec_controller`、`/dev/wheeltec_lidar`）。

临时测试：

```bash
ros2 launch turn_on_wheeltec_robot base.launch.py serial_port:=/dev/ttyUSB0
```

## 13. RPLIDAR A1M8 接入

A1M8 是 2D 激光雷达，115200 bit/s 串口，通过 `/dev/wheeltec_lidar` 接入。固件支持 Standard、Express、Boost、Stability，工程默认使用兼容性最高的 Standard。

```bash
ros2 launch turn_on_wheeltec_robot rplidar_a1.launch.py \
  serial_port:=/dev/wheeltec_lidar

ros2 topic hz /scan
```

Mini 麦克纳姆车型默认采用随车配置的 `x=0.06`、`z=0.20`、`yaw=3.14159`；雷达实际安装位姿仍须实测并在 RViz 中复核：

```bash
ros2 launch turn_on_wheeltec_robot slam_navigation.launch.py \
  laser_x:=0.06 laser_y:=0.00 laser_z:=0.20 laser_yaw:=3.14159
```

若 Web 控制台显示的激光方向与地图不一致，**首先核对 `laser_yaw`**（网关侧可用环境变量 `RESCUE_LASER_YAW_OFFSET` 覆盖）。

## 14. 救援场景传感器建议

### 14.1 基础闭环

| 传感器 | 作用 | 当前工程状态 |
| --- | --- | --- |
| 霍尔编码器 | 短时轮速与里程计 | STM32 已有 |
| 6 轴 IMU | 角速度、加速度和短时姿态约束 | STM32 已有，当前回传原始量 |
| 2D 激光雷达 | 平面避障和激光 SLAM | A1M8，`/scan`，驱动已内置 |
| RGB-D 相机 | 视觉回环、深度障碍和目标跟踪 | Astra Pro，launch 已接入 |

### 14.2 厂房救援推荐加固

普通 RGB-D 相机不应作为救援环境唯一的定位或避障传感器。烟尘、黑暗、强逆光、反光金属、热源和水雾都会使深度图或视觉特征退化。建议按预算增加：

- 3D 激光雷达：优先级最高，提供对烟尘和弱光更稳定的几何信息；RTAB-Map 可接入 3D 点云或 2D 投影。
- 热成像相机：搜索人员和高温区域，发布独立检测结果，不直接替代导航定位。
- 气体传感器：CO、CO2、VOC、氧气和可燃气体，用于风险评估与返航策略。
- UWB 定位基站或无线测距：厂房遮挡、重复纹理或烟尘导致 SLAM 退化时提供全局约束。
- 独立急停、碰撞条和安全遥控链路：不依赖 ROS2 进程，硬件层切断电机使能。
- 电池、电流和温度监测：支持低电量返航、过流保护和热失控预警。

如果只能增加一种定位相关传感器，优先选择带 IMU 的 3D 激光雷达；如果任务重点是找人，增加热成像相机和气体传感器，但仍保留激光雷达作为避障主传感器。

### 14.3 救援系统安全边界

该系统适合人在回路的实验和辅助侦察，不应直接视为消防或生命安全认证设备。必须提供人工接管、急停、失联停车、低电量停车、传感器失效降级和通信日志；正式部署前应在烟雾、弱光、反光、狭窄通道和动态障碍物条件下做分级测试。

## 15. 常见问题

### 15.1 串口打不开

```bash
ls -l /dev/wheeltec_controller
groups
sudo usermod -aG dialout $USER
```

加入 `dialout` 后需重新登录。**不要使用 `sudo ros2 launch` 绕过设备权限。**

### 15.2 有 /cmd_vel 但小车不动

- 等待 STM32 完成约 10 秒 IMU 校准。
- 检查电池是否高于 10 V。
- 检查硬件使能开关与 `/chassis_enabled`。
- 检查 ROS2 `model` 与 STM32 电位器档位。

### 15.3 RTAB-Map 没有输出 /map

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /camera/color/camera_info
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_tools view_frames
```

RGB、深度和相机内参时间戳无法同步时，`rgbd_sync` 不会输出有效数据。

### 15.4 Nav2 报 map 或 TF 超时

检查 `map -> odom -> base_footprint -> camera_link` 是否完整。若相机驱动已经发布 base 到 camera 的 TF，可关闭本工程的相机静态 TF：

```bash
publish_camera_tf:=false
```

### 15.5 编译失败

```bash
cd ~/mini_car_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --event-handlers console_direct+
```

常见原因是相机第三方库（magic_enum、libuvc）未安装，见第 2.3 节。

### 15.6 Web 控制台页面空白或无画面

- 服务未起来：确认启动时已 `source` ROS2 与工作空间，缺 `rclpy` 会直接报错退出。
- 画面黑屏：查 `http://<目标机IP>:8000/api/status` 的 `video` 字段，`error` 会说明是相机未启动还是图像编码不支持（当前支持 `rgb8`/`bgr8`）。
- 地图空白：确认 RTAB-Map 已在发布 `/map`。

## 16. Git 工作流

稳定分支为 `main`，远程使用 SSH：

```bash
git switch main
git pull --ff-only
git switch -c feature/功能名称

# 修改并在目标机编译、实车测试
git diff --check
git add .
git commit -m "feat: 功能说明"

git switch main
git merge --no-ff feature/功能名称
git push origin main
```

完成 colcon 构建、串口重连、超时停车、TF、RTAB-Map 定位和 Nav2 导航验证后，再创建版本标签：

```bash
git tag -a v2.0.0-ros2 -m "ROS2 Humble 与 RTAB-Map/Nav2 迁移版"
git push origin v2.0.0-ros2
```

`astra_camera`、`astra_camera_msgs`、`rplidar_ros` 是仓库内置的第三方源码，升级时应记录上游仓库、分支/提交和许可变化，**不得删除其 LICENSE 文件**。

## 17. 当前限制

- 开发机（Windows）无 ROS2 Humble，只能做静态语法与结构验证；`colcon build` 与实车行为必须在 Ubuntu 22.04 / Humble 上验证。
- Web 控制台桥接层固定依赖 `rclpy`，无法在无 ROS2 环境运行期自检，全部运行时验证需在目标机完成。
- `/cmd_vel` 由 Web 网关、Nav2、KCF 三方发布，**当前没有部署 twist_mux 仲裁**；接实车前必须补上。
- STM32 侧仍建议增加独立通信看门狗（树莓派死机时 ROS2 无法发零速度）。
- Web 控制台实时画面当前仅支持 `rgb8`/`bgr8` 未压缩编码；若相机发布 `mjpeg` 等压缩格式需改用 `cv_bridge`。
- Nav2 参数主要针对 Mini 麦克纳姆底盘，其他车型需要实车调参。
- ROS1 多点导航脚本没有迁移，不参与 ROS2 安装。
- 视频回传目前只有彩色画面，深度图伪彩未实现。

## 18. 参考资料

- [RTAB-Map](https://github.com/introlab/rtabmap)
- [rtabmap_ros](https://github.com/introlab/rtabmap_ros)
- [RTAB-Map 安装说明](https://github.com/introlab/rtabmap/wiki/Installation)
- [ROS2 Humble 文档](https://docs.ros.org/en/humble/)
- [Nav2 文档](https://docs.nav2.org/)
- [项目 Gitee 仓库](https://gitee.com/qbz23/mini_car_project)
- [Web 救援控制台说明](rescue_console/README.md)
