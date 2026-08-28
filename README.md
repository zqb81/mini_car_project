# Mini ROS2 小车工程

本工程包含 STM32F103VET6 底盘固件、ROS2 底盘串口桥接、KCF RGB-D 目标跟踪，以及 RTAB-Map + Nav2 建图导航配置。

智能体或新开发者接手前，请先阅读 [智能体交接文档](docs/AGENT_HANDOFF.md)、[仓库工作约束](AGENTS.md) 和 [第三方组件说明](docs/THIRD_PARTY_NOTICES.md)。

目标环境：

- Ubuntu 22.04
- ROS2 Humble
- Raspberry Pi 4 / Raspberry Pi 5
- ament_cmake + colcon
- RTAB-Map ROS2
- Nav2

STM32 通信协议保持不变，因此已经烧录的霍尔编码器版固件可以继续使用。

## 1. 系统架构

~~~text
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
~~~

| 模块 | 职责 |
| --- | --- |
| RTAB-Map | RGB-D + 激光雷达建图、回环检测、定位，发布 map -> odom |
| Nav2 | 全局规划、局部避障、行为树导航，发布 /cmd_vel |
| KCF | 目标框跟踪、目标深度估计和跟随速度计算 |
| ROS2 底盘桥接 | ROS2 话题与 STM32 二进制协议互转，发布 odom -> base_footprint |
| STM32 | 车型运动学、四路轮速 PI、编码器与 IMU 采集、电机 PWM |

## 2. 仓库结构

~~~text
mini_car/
├── F103VET6_Mini小车_STM32源码_2022.01.06(霍尔编码器)/
│   ├── USER/                         Keil 工程与程序入口
│   ├── BALANCE/                      运动学、速度 PI、车型参数
│   ├── HARDWARE/                     电机、编码器、串口、CAN、ADC
│   └── FreeRTOS/                     FreeRTOS 9
├── src/
│   ├── astra_camera/                 Astra Pro 驱动与 OpenNI2 库
│   ├── astra_camera_msgs/            Astra Pro 自定义消息与服务
│   ├── turn_on_wheeltec_robot/
│   │   ├── config/
│   │   │   ├── wheeltec_bridge.yaml  串口桥接参数
│   │   │   └── nav2_params.yaml      Nav2 参数
│   │   ├── launch/
│   │   │   ├── base.launch.py
│   │   │   ├── rtabmap_mapping.launch.py
│   │   │   ├── rtabmap_navigation.launch.py
│   │   │   └── slam_navigation.launch.py
│   │   ├── src/wheeltec_robot.cpp    ROS2 串口桥接
│   │   ├── urdf/                     车型模型
│   │   └── map/                      示例地图
│   ├── kcf_track/
│       ├── launch/kcf_tracking.launch.py
│       ├── scripts/kcf_follow.py
│       └── src/                      KCF 与 ROS2 图像节点
│   └── rplidar_ros/                  SLAMTEC RPLIDAR ROS2 驱动
└── README.md
~~~

原 ROS1 的 XML launch、send_mark.py、imageResize.py 和旧导航参数继续留在源码中供迁移对照，但 CMake 只安装 .launch.py，它们不会进入 ROS2 安装空间。

## 3. ROS2 迁移内容

底盘桥接已经从 roscpp/catkin 迁移为 rclcpp/ament：

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
- /cmd_vel 超时后主动向 STM32 发送零速度。
- 速度限幅和 int16 协议溢出保护。
- /chassis_enabled 状态话题。
- 平面里程计积分和 odom -> base_footprint TF。

KCF 节点已经迁移为 rclcpp，跟随节点已经迁移为 Python 3 + rclpy，并支持 32FC1/16UC1 深度图、ROI 边界检查和跟踪超时停车。

## 4. STM32 固件

### 4.1 基本信息

- MCU：STM32F103VET6
- 主频：72 MHz
- RTOS：FreeRTOS 9.0.0
- 控制周期：100 Hz
- PWM：TIM8 四通道，约 10 kHz
- 默认速度 PI：Kp=300、Ki=300

Keil 工程：

~~~text
F103VET6_Mini小车_STM32源码_2022.01.06(霍尔编码器)/USER/WHEELTEC.uvprojx
~~~

### 4.2 车型对应

| STM32 枚举 | ROS2 model | 车型 |
| --- | --- | --- |
| Mec_Car | mini_mec | 麦克纳姆 |
| Omni_Car | mini_omni | 三轮全向 |
| Akm_Car | mini_akm | 阿克曼 |
| Diff_Car | mini_diff | 两轮差速 |
| FourWheel_Car | mini_4wd | 四轮驱动 |

STM32 电位器档位必须与 ROS2 的 model 参数一致。

### 4.3 安全提示

STM32 原固件没有通信超时停车机制。本次 ROS2 桥接增加了上位机看门狗，但仍建议在 STM32 侧增加独立通信看门狗，因为树莓派死机时 ROS2 无法继续发送零速度。

首次电机测试必须让车轮悬空，并保证硬件使能开关随时可关闭。

## 5. 串口协议

默认设备 /dev/wheeltec_controller，115200 bit/s，8N1，对应 STM32 USART3。

### 5.1 ROS2 到 STM32

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

### 5.2 STM32 到 ROS2

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

## 6. ROS2 话题与 TF

| 名称 | 类型 | 方向 |
| --- | --- | --- |
| /cmd_vel | geometry_msgs/msg/Twist | Nav2/KCF -> 底盘 |
| /odom | nav_msgs/msg/Odometry | 底盘 -> ROS2 |
| /imu | sensor_msgs/msg/Imu | 底盘 -> ROS2 |
| /PowerVoltage | std_msgs/msg/Float32 | 底盘 -> ROS2 |
| /chassis_enabled | std_msgs/msg/Bool | 底盘 -> ROS2 |
| /scan | sensor_msgs/msg/LaserScan | 雷达 -> RTAB-Map/Nav2 |
| /map | nav_msgs/msg/OccupancyGrid | RTAB-Map -> Nav2 |

TF 链：

~~~text
map                 RTAB-Map 发布
└── odom            底盘里程计参考系
    └── base_footprint
        ├── base_link
        ├── imu_link
        └── camera_link
~~~

RTAB-Map 发布 map -> odom，底盘节点发布 odom -> base_footprint。不要启动第二个发布相同 TF 的节点。

## 7. 安装依赖

先安装 ROS2 Humble：

~~~bash
sudo apt update
sudo apt install \
  ros-humble-desktop \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-rtabmap-ros \
  ros-humble-cv-bridge \
  ros-humble-robot-state-publisher
~~~

本仓库已内置 Astra 相机和 RPLIDAR ROS2 驱动源码；目标机只需通过 `rosdep` 安装系统依赖，无需再次克隆驱动仓库。实物 Astra Pro 默认话题为：

~~~text
/camera/color/image_raw
/camera/depth/image_raw
/camera/color/camera_info
/scan
~~~

旧 ROS1 Astra 风格相机或 RealSense 等相机可以覆盖话题；RGB-D 必须使用对齐到彩色相机的深度图：

~~~bash
rgb_topic:=/camera/rgb/image_raw \
depth_topic:=/camera/depth/image \
camera_info_topic:=/camera/rgb/camera_info
~~~

## 8. 获取与构建

~~~bash
mkdir -p ~/mini_car_ws/src
cd ~/mini_car_ws/src
git clone https://gitee.com/qbz23/mini_car_project.git mini_car

cd ~/mini_car_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
~~~

可以把以下内容加入 ~/.bashrc：

~~~bash
source /opt/ros/humble/setup.bash
source ~/mini_car_ws/install/setup.bash
~~~

## 9. Astra Pro 深度相机驱动

当前实物由 USB 枚举确认是 Orbbec Astra Pro：2bc5:0403 为深度设备，2bc5:0502 为 Astra Pro FHD 彩色 UVC 设备。对应 ROS2 包已随工程纳入 src/astra_camera 和 src/astra_camera_msgs，包含随车资料提供的 OpenNI2 二进制库。第三方来源与许可边界见 docs/THIRD_PARTY_NOTICES.md。

~~~text
wheeltec_ros2/src/ros2_astra_camera/astra_camera
wheeltec_ros2/src/ros2_astra_camera/astra_camera_msgs
~~~

目标机拉取仓库后应已有：

~~~text
~/mini_car_ws/src/astra_camera
~/mini_car_ws/src/astra_camera_msgs
~~~

不要在工作空间内额外解压第二份 Astra 驱动，否则 colcon 会报 Duplicate package names。

安装构建依赖。低内存树莓派请保持单线程：

~~~bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  libgflags-dev libgoogle-glog-dev nlohmann-json3-dev \
  libusb-1.0-0-dev \
  ros-humble-image-transport \
  ros-humble-image-publisher \
  ros-humble-image-geometry \
  ros-humble-camera-info-manager \
  ros-humble-tf2-eigen \
  ros-humble-tf2-sensor-msgs

mkdir -p ~/camera_dependencies
cd ~/camera_dependencies

git clone --depth 1 --branch v0.8.0 \
  https://github.com/Neargye/magic_enum.git
cmake -S magic_enum -B magic_enum/build
cmake --build magic_enum/build --parallel 1
sudo cmake --install magic_enum/build

git clone --depth 1 https://github.com/libuvc/libuvc.git
cmake -S libuvc -B libuvc/build -DBUILD_EXAMPLES=OFF
cmake --build libuvc/build --parallel 1
sudo cmake --install libuvc/build
sudo ldconfig
~~~

编译相机包和本工程：

~~~bash
cd ~/mini_car_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

colcon build --packages-select astra_camera_msgs astra_camera \
  --symlink-install --parallel-workers 1
colcon build --packages-select turn_on_wheeltec_robot kcf_track \
  --symlink-install --parallel-workers 1
source install/setup.bash
~~~

插上 Astra Pro 后，单独验证相机。工程启动文件会开启 Astra Pro 的 UVC 彩色流、深度对齐和 RGB/Depth 同步：

~~~bash
ros2 launch turn_on_wheeltec_robot astra_pro.launch.py
~~~

另开终端检查：

~~~bash
source /opt/ros/humble/setup.bash
source ~/mini_car_ws/install/setup.bash

ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo --once /camera/color/camera_info
~~~

若相机驱动已单独启动，运行在线 SLAM 时必须禁止重复启动：

~~~bash
ros2 launch turn_on_wheeltec_robot slam_navigation.launch.py \
  start_astra_camera:=false
~~~

## 10. udev 串口配置

先确定实际设备：

~~~bash
lsusb
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
udevadm info -a -n /dev/ttyUSB0
~~~

随车脚本位于：

~~~text
src/turn_on_wheeltec_robot/scripts/wheeltec_udev.sh
~~~

脚本中的 VID、PID 和序列号是厂家示例，执行前必须与实物核对。

临时测试：

~~~bash
ros2 launch turn_on_wheeltec_robot base.launch.py serial_port:=/dev/ttyUSB0
~~~

## 11. 基础底盘测试

启动：

~~~bash
ros2 launch turn_on_wheeltec_robot base.launch.py \
  model:=mini_mec \
  serial_port:=/dev/wheeltec_controller
~~~

检查：

~~~bash
ros2 node list
ros2 topic list
ros2 topic echo --once /PowerVoltage
ros2 topic echo --once /odom
ros2 run tf2_ros tf2_echo odom base_footprint
~~~

悬空车轮后进行低速测试：

~~~bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
~~~

停止发布后，桥接节点会在默认 0.5 秒内发送零速度。

## 12. 实时 SLAM 建图与导航

实时建图和实时导航可以同时运行，但它们不是同一个节点：

~~~text
相机 + 雷达 + 编码器里程计 + IMU
                 |
                 v
RTAB-Map 增量 SLAM ---- 发布 /map 与 map -> odom
                 |
                 v
Nav2 全局/局部规划 ---- 发布 /cmd_vel
                 |
                 v
ROS2 底盘桥接 -------- 发送到 STM32 USART3
~~~

本工程新增 [slam_navigation.launch.py](src/turn_on_wheeltec_robot/launch/slam_navigation.launch.py)，它使用 RTAB-Map 增量记忆模式边建图边运行 Nav2。已有的 rtabmap_navigation.launch.py 使用定位模式，只用于加载已保存数据库后的导航。

启动实时 SLAM 导航：

~~~bash
ros2 launch turn_on_wheeltec_robot slam_navigation.launch.py \
  model:=mini_mec \
  serial_port:=/dev/wheeltec_controller \
  database_path:=$HOME/.ros/mini_car_slam.db
~~~

该入口默认自动启动 Astra Pro 与 A1M8 驱动。若已在其他终端启动相机或雷达，分别传入 start_astra_camera:=false 或 start_lidar:=false，避免重复占用设备。实时 SLAM 的最小数据闭环是：

- /camera/color/image_raw
- /camera/depth/image_raw
- /camera/color/camera_info
- /scan
- /odom
- map -> odom -> base_footprint -> camera_link

实时导航时不要启动 map_server 加载旧地图，也不要启动 AMCL 作为第二个定位源；地图由 RTAB-Map 在线发布，Nav2 的全局代价地图订阅 /map。

## 13. RTAB-Map 建图

先启动 RGB-D 相机和雷达驱动，或使用上面的实时 SLAM 入口自动启动：

~~~bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /scan
~~~

启动建图：

~~~bash
ros2 launch turn_on_wheeltec_robot rtabmap_mapping.launch.py \
  model:=mini_mec \
  database_path:=$HOME/.ros/mini_car_rtabmap.db
~~~

建图 launch 使用 -d，每次启动都会删除同一路径旧数据库。完成建图后应结束进程并备份：

~~~bash
cp ~/.ros/mini_car_rtabmap.db \
  ~/.ros/mini_car_rtabmap_$(date +%Y%m%d_%H%M%S).db
~~~

树莓派建议使用较低 RGB-D 分辨率和帧率，保持低速移动，关闭不必要的可视化，并做好散热。

## 14. RTAB-Map 数据库定位导航

导航前必须已有数据库：

~~~bash
test -f ~/.ros/mini_car_rtabmap.db && echo "数据库存在"
~~~

先启动相机与雷达，再运行：

~~~bash
ros2 launch turn_on_wheeltec_robot rtabmap_navigation.launch.py \
  model:=mini_mec \
  database_path:=$HOME/.ros/mini_car_rtabmap.db
~~~

该 launch 会：

1. 启动 STM32 底盘桥接、URDF 与静态 TF。
2. 同步 RGB 与深度图。
3. 以定位模式启动 RTAB-Map。
4. 将 RTAB-Map 命名空间内的 map 重映射为全局 /map。
5. 启动 Nav2 规划、控制、行为和速度平滑节点。

Nav2 参数位于 src/turn_on_wheeltec_robot/config/nav2_params.yaml。默认参数按 Mini 麦克纳姆底盘配置，其他车型需要调整速度空间、机器人尺寸和转弯约束。

## 15. KCF 视觉跟随

先启动 RGB-D 相机：

~~~bash
ros2 launch kcf_track kcf_tracking.launch.py \
  model:=mini_mec \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw
~~~

桌面环境中，在 KCF RGB 窗口拖动鼠标选择目标。无显示器时可以指定初始 ROI：

~~~bash
ros2 run kcf_track kcf_node --ros-args \
  -p show_window:=false \
  -p initial_roi.x:=200 \
  -p initial_roi.y:=120 \
  -p initial_roi.width:=120 \
  -p initial_roi.height:=160
~~~

KCF 与 Nav2 都会发布 /cmd_vel，不能直接同时控制底盘。并行运行时应增加 twist_mux 并制定优先级。

## 16. 救援场景传感器建议

### 16.1 基础闭环

| 传感器 | 作用 | 当前工程状态 |
| --- | --- | --- |
| 霍尔编码器 | 短时轮速与里程计 | STM32 已有 |
| 6 轴 IMU | 角速度、加速度和短时姿态约束 | STM32 已有，当前回传原始量 |
| 2D 激光雷达 | 平面避障和激光 SLAM | ROS 话题 /scan，需安装硬件驱动 |
| RGB-D 相机 | 视觉回环、深度障碍和目标跟踪 | 当前 launch 已预留 Astra 话题 |

### 16.2 RPLIDAR A1M8 接入

当前实机雷达型号为 SLAMTEC RPLIDAR A1M8。它是 2D 激光雷达，使用 115200 bit/s 串口，并通过稳定设备名 /dev/wheeltec_lidar 接入。当前 A1M8 固件支持 Standard、Express、Boost、Stability；工程默认使用兼容性最高的 Standard。

先安装官方 ROS2 驱动：

~~~bash
cd ~/mini_car_ws/src
cd ~/mini_car_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rplidar_ros --symlink-install --parallel-workers 1
source install/setup.bash
~~~

单独验证雷达：

~~~bash
ros2 launch turn_on_wheeltec_robot rplidar_a1.launch.py \
  serial_port:=/dev/wheeltec_lidar

ros2 topic hz /scan
~~~

需要提高点数时可选择 Express 或 Boost；应先以 Standard 完成方向、TF 和建图验证：

~~~bash
ros2 launch turn_on_wheeltec_robot rplidar_a1.launch.py \
  serial_port:=/dev/wheeltec_lidar \
  scan_mode:=Express
~~~

实时 SLAM 入口默认会启动 A1M8 驱动。Mini 麦克纳姆车型默认采用随车 ROS1 配置的 x=0.06、z=0.20、yaw=3.14159；雷达的实际安装位姿仍必须测量并在 RViz 中复核：

~~~bash
ros2 launch turn_on_wheeltec_robot slam_navigation.launch.py \
  laser_x:=0.06 laser_y:=0.00 laser_z:=0.20 laser_yaw:=3.14159
~~~

如果已经在另一个终端启动了雷达驱动，必须禁用重复启动：

~~~bash
ros2 launch turn_on_wheeltec_robot slam_navigation.launch.py start_lidar:=false
~~~

### 16.3 厂房救援推荐

普通 RGB-D 相机不应作为救援环境唯一的定位或避障传感器。烟尘、黑暗、强逆光、反光金属、热源和水雾都会使深度图或视觉特征退化。建议按预算增加：

- 3D 激光雷达：优先级最高，提供对烟尘和弱光更稳定的几何信息；RTAB-Map 可接入 3D 点云或 2D 投影。
- 热成像相机：搜索人员和高温区域，发布独立检测结果，不直接替代导航定位。
- 气体传感器：CO、CO2、VOC、氧气和可燃气体，用于风险评估与返航策略。
- UWB 定位基站或无线测距：厂房遮挡、重复纹理或烟尘导致 SLAM 退化时提供全局约束。
- 独立急停、碰撞条和安全遥控链路：不依赖 ROS2 进程，硬件层切断电机使能。
- 电池、电流和温度监测：支持低电量返航、过流保护和热失控预警。

如果只能增加一种定位相关传感器，优先选择带 IMU 的 3D 激光雷达；如果任务重点是找人，增加热成像相机和气体传感器，但仍保留激光雷达作为避障主传感器。

### 16.4 救援系统安全边界

该系统适合人在回路的实验和辅助侦察，不应直接视为消防或生命安全认证设备。必须提供人工接管、急停、失联停车、低电量停车、传感器失效降级和通信日志；正式部署前应在烟雾、弱光、反光、狭窄通道和动态障碍物条件下做分级测试。

## 17. 常见问题

### 17.1 串口打不开

~~~bash
ls -l /dev/wheeltec_controller
groups
sudo usermod -aG dialout $USER
~~~

加入 dialout 后需重新登录。

### 17.2 有 /cmd_vel 但小车不动

- 等待 STM32 完成约 10 秒 IMU 校准。
- 检查电池是否高于 10 V。
- 检查硬件使能开关与 /chassis_enabled。
- 检查 ROS2 model 与 STM32 电位器档位。

### 17.3 RTAB-Map 没有输出 /map

~~~bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /camera/color/camera_info
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_tools view_frames
~~~

RGB、深度和相机内参时间戳无法同步时，rgbd_sync 不会输出有效数据。

### 17.4 Nav2 报 map 或 TF 超时

检查 map -> odom -> base_footprint -> camera_link 是否完整。若相机驱动已经发布 base 到 camera 的 TF，可关闭本工程的相机静态 TF：

~~~bash
publish_camera_tf:=false
~~~

### 17.5 编译失败

~~~bash
cd ~/mini_car_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --event-handlers console_direct+
~~~

本仓库不包含相机与雷达驱动，相关包需按硬件型号安装。

## 18. Git 工作流

~~~bash
git switch main
git pull --ff-only
git switch -c feature/功能名称

# 修改并在树莓派编译、实车测试
git diff --check
git add .
git commit -m "feat: 功能说明"

git switch main
git merge --no-ff feature/功能名称
git push origin main
~~~

完成 colcon 构建、串口重连、超时停车、TF、RTAB-Map 定位和 Nav2 导航验证后，再创建版本标签：

~~~bash
git tag -a v2.0.0-ros2 -m "ROS2 Humble 与 RTAB-Map/Nav2 迁移版"
git push origin v2.0.0-ros2
~~~

## 19. 当前限制

- 当前 Windows 工作机没有 ROS2 Humble，已完成静态语法与结构验证，最终 colcon build 必须在 Ubuntu 22.04 / Humble 上执行。
- 相机、雷达驱动未纳入仓库。
- Nav2 参数主要针对 Mini 麦克纳姆底盘，其他车型需要实车调参。
- ROS1 多点导航脚本没有迁移，不参与 ROS2 安装。
- STM32 侧仍建议增加独立通信看门狗。
- 实时 SLAM 需要相机、雷达和各自 ROS2 驱动；本仓库只提供桥接和算法启动配置。

## 20. 参考资料

- [RTAB-Map](https://github.com/introlab/rtabmap)
- [rtabmap_ros](https://github.com/introlab/rtabmap_ros)
- [RTAB-Map 安装说明](https://github.com/introlab/rtabmap/wiki/Installation)
- [ROS2 Humble 文档](https://docs.ros.org/en/humble/)
- [Nav2 文档](https://docs.nav2.org/)
- [项目 Gitee 仓库](https://gitee.com/qbz23/mini_car_project)
