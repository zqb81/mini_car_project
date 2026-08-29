# Mini ROS2 小车智能体交接文档

更新时间：2026-08-29

本文面向接手本仓库的开发智能体。目标是让接手者先理解系统边界与当前验证状态，再进行修改，避免重复踩过的设备、ROS1/ROS2、TF 和驱动问题。

> 目标机上按分层顺序做功能验收时，请配合 [目标机验证清单](TARGET_VERIFICATION.md)
> 使用——它给出每层的命令、预期结果与失败排查。

## 1. 当前目标

项目正在实现厂房救援场景下的移动机器人原型，已形成「搜索 → 检测 → 规划 → 逼近」的完整链路：

- STM32 实时控制四轮麦克纳姆底盘。
- RPLIDAR A1M8 提供 2D 激光扫描。
- Orbbec Astra Pro 提供 RGB-D。
- RTAB-Map 在线增量 SLAM，发布 /map 和 map -> odom。
- Nav2 消费在线地图并发布 /cmd_vel。
- twist_mux 仲裁 Nav2 / KCF 跟随 / Web 遥操三路速度指令。
- Web 救援控制台（FastAPI 网关 + 浏览器页面）：实时地图、遥操、MJPEG 彩色画面。
- KCF 视觉跟踪：常驻跟随 + 两阶段融合跟随（Nav2 导航 + 视觉伺服）。
- YOLO 自主目标检测（rescue_perception）：发现画面中的人/物体并输出 map 系位姿。
- target_fusion 决策层：按置信度分级（自动/待确认/丢弃）+ 稳定性校验。
- search_coordinator 编排层：目标不在视野时探索未知区域，发现目标即让位。

系统数据流：

~~~text
STM32 编码器 ----------> /odom ----------------------+
STM32 MPU9250 ---------> /imu                        |
RPLIDAR A1M8 ----------> /scan                       |
Astra Pro RGB ---------> /camera/color/image_raw    |
Astra Pro Depth -------> /camera/depth/image_raw    |
                                                    v
                                          RTAB-Map 在线 SLAM
                                                    |
                                      /map + map -> odom
                                                    |
                                    +---------------+
                                    v               v
                             search_coordinator  Nav2
                             (探索开关/让位)       |
                                    |              v
                                    |        /cmd_vel (仲裁输入, 优先级 10)
                                    |              |
                                    |       twist_mux ─> /cmd_vel_muxed ─> wheeltec_robot_node
                                    |              ^
                                    |              | /cmd_vel_teleop (Web 遥操, 优先级 100)
                                    |              | /cmd_vel_kcf (KCF 跟随, 优先级 50)
                                    |              |
   detect_target ─> target_fusion ─> FollowTarget ┘
   (YOLO+深度投影)  (分级+稳定性)    (Nav2+视觉伺服)
~~~

（上位机另有 rescue_console 网关，把 /map、/scan、/cmd_vel_teleop、MJPEG
画面封装成 HTTP/WebSocket 给浏览器，客户端不依赖 ROS。）


## 2. 仓库和分支

- 仓库：https://gitee.com/qbz23/mini_car_project.git
- 稳定分支：main
- ROS2 迁移历史分支：feature/ros2-rtabmap
- 当前驱动内置基线提交：3dd8e5f，纳入 Astra 与 RPLIDAR 源码；此前 e7f6a82 完成 Astra Pro 彩色 UVC 适配。

主要目录：

~~~text
F103VET6_Mini小车_STM32源码_2022.01.06(霍尔编码器)/
src/turn_on_wheeltec_robot/     底盘桥接、URDF、RTAB-Map/Nav2 启动配置、twist_mux 配置
src/kcf_track/                  KCF 跟踪 + 两阶段融合跟随（action 接口与服务器）
src/rescue_perception/          自主检测、决策融合、搜索编排
src/astra_camera/               内置第三方 Astra 驱动
src/astra_camera_msgs/          内置第三方 Astra 消息
src/rplidar_ros/                内置第三方雷达驱动
rescue_console/                 Web 救援控制台（FastAPI 网关 + 浏览器页面）
docs/                           交接文档、第三方说明、目标机验证清单
~~~

目标机当前把仓库根目录直接作为 colcon 工作空间：

~~~text
~/mini_car_ws/.git
~/mini_car_ws/src
~/mini_car_ws/build
~/mini_car_ws/install
~/mini_car_ws/log
~~~

不要再克隆到 ~/mini_car_ws/src/mini_car，否则容易产生重复包。

## 3. 已确认硬件

### 3.1 STM32 控制板

- USB-UART：Silicon Labs CP2102。
- VID/PID：10c4:ea60。
- 序列号：0002。
- 稳定路径：/dev/wheeltec_controller。
- STM32 通信口：USART3，115200，8N1。
- 回传 /odom 已在目标机验证，约 20 Hz。
- /PowerVoltage、/imu、/chassis_enabled 由同一 24 字节状态帧生成。

### 3.2 激光雷达

- 型号：SLAMTEC RPLIDAR A1M8。
- USB-UART：CP2102。
- 序列号：0001。
- 稳定路径：/dev/wheeltec_lidar。
- 波特率：115200。
- 已验证固件：1.28，硬件修订 5，健康状态 OK。
- Standard 模式 /scan 已验证约 7.6 Hz。
- 支持模式：Standard、Express、Boost、Stability。
- 工程默认 Standard，避免旧默认 Sensitivity 不受支持。

### 3.3 深度相机

- 实物：Orbbec Astra Pro，而不是 Astra S。
- USB 2bc5:0403：Astra Pro 深度设备。
- USB 2bc5:0502：Astra Pro FHD 彩色 UVC 设备。
- 深度话题已验证约 29.5 Hz。
- 旧 Astra S 配置下彩色话题有 publisher 但没有帧。
- 已新增 astra_pro.launch.py，启用 uvc_camera.enable、depth_align 和 color_depth_synchronization。
- Astra Pro 修复提交后，彩色流是否恢复尚需目标机复验。

### 3.4 普通 USB 单目相机

曾测试一个普通 UVC 相机：

- MJPG 640x480@30，YUYV 640x480@20。
- v4l2_camera 的 MJPG 路径崩溃。
- usb_cam 的 raw_mjpeg 路径也出现段错误。
- 它不是当前 SLAM 的深度相机，不要再将其话题接入 RGB-D RTAB-Map。

## 4. 当前验证状态

| 模块 | 状态 | 证据/备注 |
| --- | --- | --- |
| ROS2 底盘包编译 | 已通过 | 目标机可启动 base.launch.py |
| STM32 串口打开 | 已通过 | /dev/wheeltec_controller |
| STM32 上行协议 | 已通过 | /odom 约 20 Hz |
| cmd_vel ROS2 看门狗 | 已观察 | 0.5 秒未收到命令发送零速度 |
| RPLIDAR 驱动编译 | 已通过 | 单线程约 3 分 44 秒 |
| RPLIDAR /scan | 已通过 | Standard 约 7.6 Hz |
| Astra 深度流 | 已通过 | /camera/depth/image_raw 约 29.5 Hz |
| Astra 彩色流 | 待复验 | 见验证清单第 3 层 |
| RGB-D 对齐与同步 | 待复验 | 必须检查 RGB、Depth frame_id 和时间戳 |
| /rtabmap/rgbd_image | 待验证 | 依赖彩色流修复 |
| /map | 待验证 | 在线整栈尚未验收 |
| map -> odom | 待验证 | 曾因 STM32 串口缺失而超时 |
| Nav2 生命周期 active | 待验证 | controller/planner/bt_navigator |
| Nav2 目标运动 | 待验证 | 无目标时车辆不动是正常行为 |
| KCF 运行 | 待验证 | 已迁移代码，未完成实机验收 |

2026-08-29 新增能力的验证状态（代码已全部推送 main，逻辑已验证，**实车待验**）：

| 模块 | 逻辑验证 | 实车验证 |
| --- | --- | --- |
| Web 救援控制台（遥测/地图/MJPEG 画面/导航下发） | 已完成 | 待验（验证清单第 7 层） |
| twist_mux 速度仲裁（优先级/超时降级/急停锁） | 已完成 | 待验（第 6 层） |
| 两阶段融合跟随（FollowTarget action） | 已完成 | 待验（第 8 层） |
| YOLO 自主目标检测 + 深度投影 | conda 环境实跑通过 | 待验（第 9 层，**深度单位需实测**） |
| target_fusion 决策（三级置信度 + 稳定性） | 桩测试通过 | 待验（第 10 层） |
| search_coordinator 搜索编排（状态机） | 桩测试通过 | 待验（第 11 层） |

## 5. ROS2 包

### 5.1 turn_on_wheeltec_robot

职责：

- 连接 STM32 串口。
- 发布 /odom、/imu、/PowerVoltage、/chassis_enabled。
- 订阅 /cmd_vel。
- 发布 odom -> base_footprint。
- 启动 URDF、雷达、相机、RTAB-Map 和 Nav2。

核心文件：

~~~text
src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp
src/turn_on_wheeltec_robot/include/wheeltec_robot.h
src/turn_on_wheeltec_robot/config/wheeltec_bridge.yaml
src/turn_on_wheeltec_robot/config/nav2_params.yaml
~~~

### 5.2 kcf_track

职责：

- `kcf_node`（C++）：订阅 /camera/color/image_raw 和 /camera/depth/image_raw，
  输出 kcf/track（`linear.x`=距离，`angular.z`=像素横坐标）。**它不发布速度**。
- `kcf_follow.py`：常驻跟随，把 kcf/track 转为速度，默认发 `cmd_vel_kcf`。
- `follow_target_server.py`：两阶段融合跟随 action 服务器（staging 用 Nav2
  导航 + servo 用视觉伺服环）。
- `kcf_control.py`：PD 控制律，两种跟随模式共用。
- 0.5 秒无跟踪数据时发布零速度。

三种速度源（Nav2、KCF 跟随、Web 遥操）已由 twist_mux 仲裁，见 5.4 节。
`kcf_tracking.launch.py` 通过 `follow_mode:=continuous|fusion|none` 选择
跟随模式——两种跟随模式都会发同一个速度话题，**不可同时启动**。

### 5.3 内置第三方驱动

目标机工作空间额外包含：

~~~text
src/rplidar_ros
src/astra_camera
src/astra_camera_msgs
~~~

- rplidar_ros 来自 SLAMTEC ros2 分支，保留其 LICENSE 和 CHANGELOG。
- Astra 两个包来自随车 humble-src-2023-12-29.zip，包含专有 OpenNI2 二进制和原始许可信息。
- 这三个包已作为 vendored source 纳入主仓库，目标机不需要额外克隆或复制。
- 临时解压目录必须放到工作空间外。colcon 会递归扫描工作空间，曾因 _vendor 与 src 中同时存在 Astra 包而报重复包。
- 第三方来源、提交和许可限制见 docs/THIRD_PARTY_NOTICES.md。

### 5.4 速度指令仲裁（twist_mux）

`slam_navigation.launch.py` 默认启用 twist_mux（`use_twist_mux:=false` 可关闭），
底盘只接收仲裁输出 `/cmd_vel_muxed`。配置见
`src/turn_on_wheeltec_robot/config/twist_mux.yaml`：

| 源 | 话题 | 优先级 |
| --- | --- | --- |
| Nav2（velocity_smoother 输出） | /cmd_vel | 10 |
| KCF 跟随 | /cmd_vel_kcf | 50 |
| Web 遥操 | /cmd_vel_teleop | 100 |
| 急停锁（Bool 置 true 屏蔽全部） | /cmd_vel_estop_lock | 255 |

每路 0.5s 超时，失效自动降级；全部失效输出零速度。**急停锁只是软件层屏蔽，
不能替代硬件急停**。依赖：`sudo apt install ros-humble-twist-mux`。

Nav2 零改动集成：Humble 的 velocity_smoother 输出就是 /cmd_vel，仲裁器直接
订阅它；底盘硬编码的 "cmd_vel" 由 base.launch.py 的 `cmd_vel_topic` 参数
remap 解决，未改 C++。

### 5.5 rescue_perception（自主检测 → 决策 → 搜索编排）

纯 Python 包（ament_python），三个节点：

| 节点 | 职责 | 关键话题 |
| --- | --- | --- |
| detect_target | YOLO 检测 + 深度投影 → map 系位姿 | `/rescue/target_pose`、`/detect_target/detections_3d`、`/detect_target/debug_image` |
| target_fusion | 三级置信度决策 + 稳定性校验，下发 FollowTarget | `/rescue/pending_target`、`/rescue/fusion_state`、`/rescue/confirm` |
| search_coordinator | 探索与接近的编排状态机 | `/rescue/search_cmd`、`/rescue/search_state`、`/explore/resume` |

要点：

- **检测只做感知不发布速度**，运动始终由 Nav2 独占。
- 置信度分级：≥`auto_conf_threshold`(0.75) 自动 / ≥`confirm_conf_threshold`(0.40)
  待确认 / 以下丢弃；`auto_mode:=false` 强制全部人工确认。
- 稳定性校验：连续 `min_stable_count`(3) 次位置在 `stability_radius`(0.5m)
  内才认定稳定，抑制闪烁误检。
- 搜索编排必须自写（无现成实现）：不协调时 explore_lite 与 FollowTarget 会
  互相抢占 Nav2。状态机 IDLE→SEARCHING→YIELDED→(resume_delay)→SEARCHING。
- 深度编码：支持 `16UC1`（毫米）/`32FC1`（米），未知编码告警跳过。
- 依赖 ultralytics（`pip install -r requirements.txt`，会拉入 torch）。

启动入口：`detect_target.launch.py`（仅检测）、`rescue_perception.launch.py`
（检测+融合）、`rescue_search.launch.py`（探索+编排）。

`explore_lite`（m-explore-ros2）**不内置仓库**，需按 README 第 5.8 节安装。

### 5.6 rescue_console（Web 救援控制台）

小车侧 FastAPI 网关 + 浏览器页面，客户端不依赖 ROS。目录在仓库根 `rescue_console/`：

- `server/app.py`：REST + WebSocket 遥测 + MJPEG 视频流。
- `server/bridge.py`：RosBridge（rclpy），订阅 /odom /scan /map 等，发布
  速度到 `/cmd_vel_teleop`（环境变量 `RESCUE_CMD_VEL_TOPIC` 可覆盖）。
- `web/index.html`：实时地图、激光、轨迹、遥操、MJPEG 画面、导航目标。
- `deploy/`：可选 systemd 开机自启。

详见 rescue_console/README.md。**若关闭仲裁，必须把网关输出话题改回
`/cmd_vel`**，否则遥控无效。

## 6. 启动入口

| 文件 | 用途 | 是否自动启动传感器 |
| --- | --- | --- |
| base.launch.py | 仅底盘、URDF、基础 TF | 否 |
| rplidar_a1.launch.py | A1M8 与 base_footprint -> laser | 雷达 |
| astra_s.launch.py | Astra S 单设备模式，保留兼容（**本设备是 Astra Pro，勿用**） | 相机 |
| astra_pro.launch.py | Astra Pro 深度 + UVC 彩色 | 相机 |
| rtabmap_mapping.launch.py | 只运行 RTAB-Map 建图 | 否 |
| rtabmap_navigation.launch.py | 已有数据库定位 + Nav2 | 否 |
| slam_navigation.launch.py | 在线增量 SLAM + Nav2 + twist_mux 仲裁 | 底盘、A1M8、Astra Pro |
| kcf_tracking.launch.py | KCF 跟踪（follow_mode:=continuous/fusion/none） | 底盘，不启动相机 |
| rescue_console | Web 网关（venv 内 uvicorn，非 launch） | 否 |
| rescue_perception.launch.py | 目标检测 + 融合决策 | 否（复用已启动的相机） |
| rescue_search.launch.py | explore_lite 探索 + 搜索编排 | 否 |

主入口：

~~~bash
# 前提：已 apt install ros-humble-twist-mux（默认启用仲裁，见 5.4 节）
ros2 launch turn_on_wheeltec_robot slam_navigation.launch.py \
  model:=mini_mec \
  serial_port:=/dev/wheeltec_controller
~~~

这个命令不会自动让车行驶。只有手动 /cmd_vel_teleop（Web 控制台）、/cmd_vel
（Nav2）或 Nav2 Goal 才会产生运动。**关闭仲裁需显式 `use_twist_mux:=false`**，
此时底盘直接订阅 Nav2 的 /cmd_vel。

如果传感器已单独启动：

~~~bash
ros2 launch turn_on_wheeltec_robot slam_navigation.launch.py \
  start_lidar:=false \
  start_astra_camera:=false
~~~

## 7. 话题契约

| 话题 | 类型 | 典型频率 | 生产者 |
| --- | --- | ---: | --- |
| /odom | nav_msgs/msg/Odometry | 20 Hz | wheeltec_robot_node |
| /imu | sensor_msgs/msg/Imu | 20 Hz | wheeltec_robot_node |
| /PowerVoltage | std_msgs/msg/Float32 | 20 Hz | wheeltec_robot_node |
| /chassis_enabled | std_msgs/msg/Bool | 20 Hz | wheeltec_robot_node |
| /scan | sensor_msgs/msg/LaserScan | 7.6 Hz | rplidar_node |
| /camera/color/image_raw | sensor_msgs/msg/Image | 目标 30 Hz | astra_camera |
| /camera/depth/image_raw | sensor_msgs/msg/Image | 29.5 Hz | astra_camera |
| /camera/color/camera_info | sensor_msgs/msg/CameraInfo | 随帧 | astra_camera |
| /rtabmap/rgbd_image | rtabmap_msgs/msg/RGBDImage | 待验证 | rgbd_sync |
| /map | nav_msgs/msg/OccupancyGrid | 动态 | RTAB-Map |
| /cmd_vel | geometry_msgs/msg/Twist | Nav2 控制周期 | velocity_smoother（仲裁输入，优先级 10） |
| /cmd_vel_kcf | geometry_msgs/msg/Twist | 跟随周期 | kcf_follow / follow_target_server（仲裁输入，优先级 50） |
| /cmd_vel_teleop | geometry_msgs/msg/Twist | 遥操周期 | rescue_console RosBridge（仲裁输入，优先级 100） |
| /cmd_vel_estop_lock | std_msgs/msg/Bool | 事件 | 急停锁，置 true 屏蔽全部源 |
| /cmd_vel_muxed | geometry_msgs/msg/Twist | 仲裁周期 | twist_mux（**底盘唯一接收的指令**） |
| kcf/track | geometry_msgs/msg/Twist | 跟踪周期 | kcf_node（linear.x=距离，angular.z=像素） |
| /follow_target | kcf_track/action/FollowTarget | 事件 | follow_target_server |
| /rescue/target_pose | geometry_msgs/msg/PoseStamped | 检测周期 | detect_target |
| /rescue/target_roi | sensor_msgs/msg/RegionOfInterest | 检测周期 | detect_target -> kcf_node 自动初始化跟踪 |
| /detect_target/detections_3d | vision_msgs/msg/Detection3DArray | 检测周期 | detect_target |
| /detect_target/debug_image | sensor_msgs/msg/Image | 检测周期 | detect_target |
| /rescue/pending_target | geometry_msgs/msg/PoseStamped | 事件 | target_fusion（待人工确认） |
| /rescue/fusion_state | std_msgs/msg/String | 事件 | target_fusion（idle/pending/following） |
| /rescue/confirm | std_msgs/msg/Bool | 事件 | 人工确认待确认目标 |
| /rescue/search_cmd | std_msgs/msg/Bool | 事件 | 外部启停搜索 |
| /rescue/search_state | std_msgs/msg/String | 1 Hz | search_coordinator（idle/searching/yielded） |
| /explore/resume | std_msgs/msg/Bool | 1 Hz | search_coordinator 控制 explore_lite |

## 8. TF 契约

唯一允许的主链：

~~~text
map
└── odom
    └── base_footprint
        ├── base_link
        ├── imu_link
        ├── laser
        └── camera_link
            ├── camera_depth_frame
            └── camera_color_frame
~~~

责任边界：

- RTAB-Map：map -> odom。
- wheeltec_robot_node：odom -> base_footprint。
- base.launch.py：base_footprint -> base_link、imu_link、camera_link。
- rplidar_a1.launch.py：base_footprint -> laser。
- Astra 驱动：camera_link 内部光学 TF。

若相机驱动已经发布 base_footprint -> camera_link，应使用 publish_camera_tf:=false 禁止重复 TF。

## 9. 设备映射

udev 规则应表达：

~~~text
CP2102 serial 0001 -> /dev/wheeltec_lidar
CP2102 serial 0002 -> /dev/wheeltec_controller
~~~

ttyUSB 编号会随插拔顺序变化。诊断必须看序列号：

~~~bash
for port in /dev/ttyUSB*; do
  echo "===== $port ====="
  udevadm info -q property -n "$port" | \
    grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_MODEL='
done
~~~

Astra Pro：

~~~text
2bc5:0403 depth
2bc5:0502 color UVC
~~~

## 10. 串口协议

ROS2 到 STM32，共 11 字节：

~~~text
0      0x7B
1..2   保留
3..4   Vx * 1000，大端 int16
5..6   Vy * 1000，大端 int16
7..8   Vz * 1000，大端 int16
9      0..8 XOR
10     0x7D
~~~

STM32 到 ROS2，共 24 字节：

~~~text
0       0x7B
1       软件停止标志
2..7    Vx、Vy、Vz
8..13   加速度原始量
14..19  角速度原始量
20..21  电压 * 1000
22      0..21 XOR
23      0x7D
~~~

## 11. 目标机构建

每个新终端：

~~~bash
cd ~/mini_car_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
~~~

更新主工程：

~~~bash
cd ~/mini_car_ws
git pull --ff-only
source /opt/ros/humble/setup.bash
colcon build --packages-select turn_on_wheeltec_robot kcf_track rescue_perception \
  --symlink-install --parallel-workers 1
source install/setup.bash
~~~

新增依赖（首次）：

~~~bash
sudo apt install -y ros-humble-twist-mux
pip3 install -r src/rescue_perception/requirements.txt   # ultralytics，会拉入 torch
python3 -m venv rescue_console/venv
./rescue_console/venv/bin/pip install -r rescue_console/server/requirements.txt
~~~

完整依赖检查：

~~~bash
rosdep install --from-paths src --ignore-src -r -y
~~~

树莓派内存有限，rplidar_ros 并行构建曾失败。应使用：

~~~bash
export CMAKE_BUILD_PARALLEL_LEVEL=1
colcon build --packages-select rplidar_ros \
  --parallel-workers 1 \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --event-handlers console_direct+
~~~

不要在失败构建后立即 source install/setup.bash；先确认对应包显示 Finished。

## 12. 分层验收顺序

> 详细的命令、预期结果与失败排查见 [目标机验证清单](TARGET_VERIFICATION.md)
> （第 0–11 层）。本节保留精简要点；清单新增了仲裁、Web 控制台、融合跟随、
> 检测、融合、搜索六层，本节的 12.6 之后是旧版精简，验收时以清单为准。

### 12.1 设备

~~~bash
ls -l /dev/wheeltec_controller /dev/wheeltec_lidar
lsusb
~~~

### 12.2 底盘

~~~bash
ros2 launch turn_on_wheeltec_robot base.launch.py \
  model:=mini_mec \
  serial_port:=/dev/wheeltec_controller

timeout 8 ros2 topic hz /odom
~~~

### 12.3 雷达

~~~bash
ros2 launch turn_on_wheeltec_robot rplidar_a1.launch.py \
  serial_port:=/dev/wheeltec_lidar

timeout 8 ros2 topic hz /scan
~~~

### 12.4 Astra Pro

先测试厂家配置：

~~~bash
ros2 launch astra_camera astra_pro.launch.py
~~~

再测试项目封装：

~~~bash
ros2 launch turn_on_wheeltec_robot astra_pro.launch.py
~~~

验收：

~~~bash
timeout 8 ros2 topic hz /camera/color/image_raw
timeout 8 ros2 topic hz /camera/depth/image_raw
ros2 topic echo --once /camera/color/camera_info
~~~

rqt_image_view 应选择 raw 话题，不能选 compressed。空 compressed 流曾触发 OpenCV imdecode 断言。

### 12.5 RGB-D 同步与 SLAM

~~~bash
timeout 8 ros2 topic hz /rtabmap/rgbd_image
timeout 8 ros2 topic hz /map
ros2 run tf2_ros tf2_echo map odom
~~~

### 12.6 Nav2

~~~bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
~~~

三者必须为 active。无导航目标时车辆保持静止是正常行为。

### 12.7 低速底盘测试

车轮悬空后：

~~~bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
~~~

停止发布后，ROS2 看门狗应在 0.5 秒内发送零速度。

## 13. 已知问题和排障经验

### 13.1 重复包

colcon 会递归扫描工作空间。曾出现：

~~~text
_vendor/.../astra_camera
src/astra_camera
~~~

导致 Duplicate package names。临时解压目录必须移到 ~/mini_car_ws 之外。

### 13.2 串口存在但路径错误

- No such file or directory：软链接未创建或设备没插。
- Permission denied：当前用户不在 dialout，添加后必须重新登录。
- 不要 sudo ros2 launch。

### 13.3 map TF 超时

planner 等待 base_footprint -> map 超时通常不是 Nav2 本身故障。依次确认：

1. /odom 是否发布。
2. odom -> base_footprint 是否存在。
3. RTAB-Map 是否收到 RGB-D 与 /scan。
4. RTAB-Map 是否发布 map -> odom。

曾因 STM32 USB 未连接导致整条 TF 链缺失。

### 13.4 Astra list_devices_node 段错误

它曾在打印 Astra 设备 URI 后段错误。设备枚举信息仍有效，不要把这个工具的退出状态当作相机数据流验收；以实际图像话题频率为准。

### 13.5 Astra Pro 彩色流

Astra Pro 将深度和 RGB 分成两个 USB 设备。Astra S 配置只启动深度，彩色 publisher 存在但不产生帧。必须启用 uvc_camera.enable，当前 astra_pro.launch.py 已处理。

### 13.6 rqt_image_view

选择空的 compressed 话题会反复出现 OpenCV imdecode !buf.empty。验收时选择：

~~~text
/camera/color/image_raw
/camera/depth/image_raw
~~~

### 13.7 RPLIDAR 扫描模式

A1M8 不支持 Sensitivity。当前默认 Standard。出现 Can not start scan 时先查看驱动打印的 supported modes。

## 14. 下一步优先级

已完成（2026-08-29）：twist_mux 仲裁、Web 救援控制台、两阶段融合跟随、
自主检测、决策融合、搜索编排——代码全部推送 main，逻辑验证通过。

实车验收（按 [目标机验证清单](TARGET_VERIFICATION.md) 分层执行）：

1. 在目标机拉取最新 main，按第 11 节安装新增依赖并重新构建全部包。
2. 启动 astra_pro.launch.py，确认彩色流恢复、depth_align 后 frame_id 与时间戳合理。
3. 启动 slam_navigation.launch.py，确认 /rtabmap/rgbd_image、/map 和 map -> odom。
4. 确认 Nav2 生命周期 active，在 RViz2 发送目标并低速测试。
5. 验证 twist_mux：优先级切换、超时降级、急停锁（清单第 6 层）。
6. 验证 Web 控制台：遥测、地图、MJPEG 画面、遥控（第 7 层）。
7. 验证两阶段融合跟随（第 8 层）；目标丢失停车与两阶段切换。
8. **实测深度单位与检测精度**（第 3、9 层）——这是开发机无法验证的假设。
9. 用卷尺实测 base_footprint 到 laser、camera_link 的外参，替换随车默认值。
10. 验证融合决策（先 auto_mode:=false 验人工确认，第 10 层）与自主搜索（第 11 层）。
11. 录制 rosbag2，覆盖 /odom、/imu、/scan、RGB、Depth、/tf、/tf_static、
    /cmd_vel_muxed、/map 及各 /rescue/* 话题。
12. 在 STM32 侧实现独立通信看门狗。
13. 厂房救援正式测试前增加硬件急停、碰撞条、低电量/失联停车和人工接管。

## 15. 诊断信息采集

接手者遇到整栈问题时，优先收集：

~~~bash
git status --short --branch
git log -5 --oneline
ros2 node list
ros2 topic list -t
ros2 topic info -v /cmd_vel_muxed
ros2 topic info -v /cmd_vel_teleop
ros2 topic info -v /camera/color/image_raw
ros2 topic info -v /camera/depth/image_raw
ros2 action list
ros2 action send_goal --help 2>/dev/null | head -1   # 检查 follow_target / navigate_to_pose 可用
ros2 run tf2_tools view_frames
ls -l /dev/wheeltec_controller /dev/wheeltec_lidar
lsusb
~~~

"遥控/仲裁类"问题额外收集：

~~~bash
ros2 topic echo --once /rescue/fusion_state
ros2 topic echo --once /rescue/search_state
ros2 topic echo --once /cmd_vel_estop_lock
~~~

频率使用 timeout 分别测量，避免第一个 ros2 topic hz 阻塞后续命令：

~~~bash
for topic in /odom /scan /camera/color/image_raw /camera/depth/image_raw /map \
             /detect_target/detections_3d /cmd_vel_muxed; do
  echo "===== $topic ====="
  timeout 8 ros2 topic hz "$topic"
done
~~~

完整启动日志位于 ~/.ros/log。构建失败时读取 log/latest_build/包名/stderr.log，不要只粘贴大量 warning。
