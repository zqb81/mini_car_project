# Mini ROS2 小车工程（厂房救援）

面向厂房救援场景的轮式机器人工程：STM32F103VET6 底盘固件 + ROS2 串口桥接 + RTAB-Map 实时建图 + Nav2 导航 + KCF 视觉跟随，并附带一套不依赖 ROS 的 Web 救援控制台，可通过浏览器实时查看建图、遥操小车与下发导航目标。

核心价值：**救援任务无法提前建图**，因此本工程以 RTAB-Map 增量记忆模式边行驶边建图、边定位边导航，并通过 Web 控制台把实时地图、实时画面与遥控能力交给操作员。

智能体或新开发者接手前，请先阅读 [智能体交接文档](docs/AGENT_HANDOFF.md)、[仓库工作约束](AGENTS.md) 和 [第三方组件说明](docs/THIRD_PARTY_NOTICES.md)。

> 在目标机上首次部署或验证时，请照 [目标机验证清单](docs/TARGET_VERIFICATION.md)
> 分层执行——它给出每一层的具体命令、预期结果与失败排查。**不要跳步**，
> 否则故障难以定位。

## 1. 核心功能特性

| 能力 | 说明 |
| --- | --- |
| 实时 SLAM 建图 | RTAB-Map 增量记忆模式，RGB-D + 2D 激光融合，边走边建，发布 `/map` 与 `map -> odom` |
| 实时导航 | Nav2 全局/局部规划，点击目标点即自动行驶；网关走 `NavigateToPose` action 并回传到达/失败/取消 |
| Web 救援控制台 | FastAPI 网关 + 浏览器页面：实时地图、激光扫描、行驶轨迹、手动遥操、MJPEG 实时画面 |
| 彩色实时画面 | `/camera/color/image_raw` 转 MJPEG 推流（限流 10fps），供救援遥操看路 |
| 多车型底盘 | 麦克纳姆、全向三轮、阿克曼、两轮差速、四轮驱动，STM32 与 ROS2 `model` 参数对应 |
| KCF 视觉跟随 | RGB-D 目标跟踪，两种模式：常驻跟随 / 两阶段融合（Nav2 导航 + 视觉伺服） |
| 自主目标检测 | YOLO 自主发现画面中的人/物体，经深度投影输出 map 系目标位姿，供导航规划 |
| 自主搜索 | 目标不在视野时按 frontier 探索未知区域，发现目标即让位给接近动作 |
| 速度指令仲裁 | twist_mux 按优先级合并 Nav2、KCF、Web 遥操三路指令，避免争抢底盘 |

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
│   │   ├── scripts/             wheeltec_udev.sh
│   ├── astra_camera/            Astra Pro 驱动与随车 OpenNI2 二进制库
│   ├── astra_camera_msgs/       Astra Pro 自定义消息与服务
│   ├── rplidar_ros/             SLAMTEC RPLIDAR ROS2 驱动
│   ├── kcf_track/               KCF 目标跟踪
│   │   ├── action/              FollowTarget.action（两阶段融合跟随接口）
│   │   ├── scripts/             kcf_follow.py（常驻跟随）
│   │   │                        kcf_control.py（PD 控制律，两种模式共用）
│   │   │                        follow_target_server.py（两阶段融合服务器）
│   │   ├── src/                 kcf_node（C++ 视觉跟踪，输出 kcf/track）
│   │   └── launch/              kcf_tracking.launch.py
│   └── rescue_perception/       救援目标感知（自主检测，非人工初始化）
│       ├── rescue_perception/   detect_target.py（YOLO + 深度投影）
│       │                        target_fusion.py（置信度分级 + 稳定性校验）
│       │                        search_coordinator.py（探索与接近的编排）
│       ├── config/              explore_params.yaml（frontier 探索参数）
│       └── launch/              detect_target / rescue_perception / rescue_search
├── ros1_rsc/                    ROS1 旧版迁移参考资料（不参与 ROS2 构建）
│   ├── turn_on_wheeltec_robot/  旧 XML launch、参数、示例地图、send_mark.py
│   └── kcf_track/               旧 KCF XML launch 与图像缩放脚本
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
| `astra_s.launch.py` | Astra S 相机入口 |

旧 ROS1 XML launch、ROS1 参数、示例地图及脚本已统一移至仓库根目录 `ros1_rsc/`，仅作迁移对照；它们不属于 ROS2 `src/`，不会被 colcon 构建或安装。ROS2 包的 `launch/` 目录只保留 `*.launch.py`。

## 4. 安装与构建

> 构建完成后，建议照 [目标机验证清单](docs/TARGET_VERIFICATION.md) 分层验收
> （底盘 → 雷达 → 相机 → SLAM → Nav2 → 各新增能力），不要直接跳到整栈。

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

#### 两种跟随模式（互斥，不可同时启动）

`kcf_tracking.launch.py` 通过 `follow_mode` 选择，两种模式都会向 `cmd_vel_topic` 下发指令，同时运行会互相打架：

| follow_mode | 行为 | 适用场景 |
| --- | --- | --- |
| `continuous`（默认） | `kcf_follower` 常驻，一有目标就驱动底盘 | 简单跟随，目标始终在视野内 |
| `fusion` | 只启动 `follow_target` 动作服务器，由调用方下发目标触发 | 目标尚远或需要避障 |
| `none` | 只跟踪并发布 `kcf/track`，不驱动底盘 | 上层自行消费跟踪结果 |

**两阶段融合模式**（参考 OpenNav Docking 的分阶段思路）：

```bash
ros2 launch kcf_track kcf_tracking.launch.py \
  follow_mode:=fusion \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw
```

调用 `follow_target` action 后分两阶段执行：

1. **staging**：用 Nav2 `NavigateToPose` 走到目标大致位置，这段路享有完整避障；
2. **servo**：进入视觉伺服环，KCF 持续给出目标距离与像素位置，由 PD 控制器逼近并保持距离。

任一时刻只有一个控制器活跃，因此不会与导航争抢 `/cmd_vel`。命令示例：

```bash
ros2 action send_goal /follow_target kcf_track/action/FollowTarget \
  "{use_staging_pose: true,
    staging_pose: {header: {frame_id: map},
                   pose: {position: {x: 2.0, y: 1.0, z: 0.0},
                          orientation: {w: 1.0}}},
    target_distance: 1.2,
    servo_timeout: 60.0}"
```

目标丢失、超时或被抢占时都会**先停车再返回结果**，错误码见 `FollowTarget.action` 注释。

### 5.7 自主目标检测

> **为什么需要这一层**：KCF 是跟踪器不是检测器，必须先由人框选目标才能跟
> （见 5.6）。而救援恰恰是**不知道人在哪**才需要机器人去找。本节点让相机
> 自主发现目标，输出导航可直接消费的 map 系位姿。

先安装依赖（会拉入 torch，体积较大，建议预留时间）：

```bash
pip3 install -r ~/mini_car_ws/src/rescue_perception/requirements.txt
```

启动（相机与 SLAM 需已在运行）：

```bash
ros2 launch rescue_perception detect_target.launch.py \
  target_classes:=person \
  conf_threshold:=0.5 \
  min_interval:=0.5
```

常用参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `model` | `yolov8n.pt` | YOLO 权重，nano 版在 CPU 上最轻 |
| `target_classes` | `person` | COCO 类别名，多个用逗号分隔 |
| `conf_threshold` | `0.5` | 置信度阈值 |
| `min_interval` | `0.5` | 检测最小间隔（秒），**树莓派上用于限流** |
| `max_depth` | `8.0` | 有效测距上限（米） |
| `target_frame` | `map` | 输出位姿所在坐标系 |

输出：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/rescue/target_pose` | geometry_msgs/PoseStamped | map 系目标位姿（取置信度最高的目标） |
| `/rescue/target_roi` | sensor_msgs/RegionOfInterest | 最佳检测框，自动交给 KCF 初始化跟踪 |
| `~/detections_3d` | vision_msgs/Detection3DArray | 全部 3D 检测结果，供 RViz 显示 |
| `~/debug_image` | sensor_msgs/Image | 画了检测框与测距的图像 |

验证：

```bash
ros2 topic echo /rescue/target_pose
ros2 run rqt_image_view rqt_image_view   # 选 /detect_target/debug_image
```

**本节点只做感知，不发布任何速度指令**——运动始终由 Nav2 独占。

#### 坐标链路与两个前提

1. **深度图必须已对齐到彩色视角**（`astra_pro.launch.py` 的 `depth_align: true`）。
   未对齐时彩色像素与深度像素不对应，测距会偏；此时节点会按分辨率比例做
   兜底缩放，但那只是粗略可用。
2. **TF 链路必须完整**：`map -> odom -> base_footprint -> camera`。
   RTAB-Map 提供 `map -> odom`，缺失时节点会告警并跳过该帧。

深度单位为毫米（Astra 默认 `16UC1`），节点已按编码自动换算；未知编码会
告警并跳过，避免把毫米当米用导致测距错 1000 倍。

#### 算力约束

树莓派 CPU 上 YOLOv8n 约**个位数 FPS**，故默认限流到 2Hz。如需提速，按成本
递增：降输入分辨率 → Coral TPU / Hailo 加速棒 → 换用自带 VPU 的相机（如
OAK-D，检测在相机端完成，宿主零负担）。

#### 检测与导航的自动衔接（target_fusion）

`detect_target` 只回答「画面里有什么、在哪」，不决定要不要过去。融合节点
补上决策层，自动把检测结果变成 `FollowTarget` 的 `staging_pose`：

检测节点同时发布 `/rescue/target_roi`，KCF 收到非空框后会在下一帧 RGB 上自动
初始化跟踪，因此自主检测不再要求人工鼠标框选。导航 staging 点沿机器人到目标的
方向退让，默认与目标保持 2.0 米，再由视觉伺服逼近到 `follow_distance`。

```bash
# 完整链路：检测 + 融合决策（需 kcf_track 以 follow_mode:=fusion 启动）
ros2 launch rescue_perception rescue_perception.launch.py

# 人工确认（对当前待确认目标下发导航）
ros2 topic pub --once /rescue/confirm std_msgs/msg/Bool "{data: true}"
```

三级置信度策略（阈值可配）：

| 置信度 | 行为 |
| --- | --- |
| ≥ `auto_conf_threshold`（默认 0.75） | 目标稳定后自动下发导航 |
| ≥ `confirm_conf_threshold`（默认 0.40） | 记为待确认，发布 `/rescue/pending_target`，等人工确认 |
| < 0.40 | 丢弃 |

**稳定性校验**：单帧检测可能是闪烁误检，因此要求连续 `min_stable_count`
（默认 3）次检测位置在 `stability_radius`（默认 0.5m）内，才认定为稳定目标。
位置跳变会重新累计。

救援场景误检代价高（机器人可能冲向错误的「人」而错过真正目标），因此提供
`auto_mode:=false` 开关——关闭后**所有**目标都需人工确认，适合调试或高风险
环境。

融合节点同样不发布速度指令，只下发 action。

### 5.8 自主搜索（目标不在视野时）

检测只在目标进入相机视野时有效，而救援恰恰不知道人在哪。本节点补上
「目标不在视野时怎么办」：按 frontier 主动探索未知区域，边建图边搜索；
一旦发现目标就暂停探索，把底盘控制权让给 `FollowTarget`。

**为什么需要编排层**：`explore_lite` 只管探索、不知道检测的存在；
`target_fusion` 只管目标决策、不知道探索的存在。若不协调，两路目标会互相
抢占 Nav2，表现为机器人在「去目标」和「去 frontier」之间反复横跳。
`search_coordinator` 就是把两者粘起来的编排层：

```text
/rescue/fusion_state ─> search_coordinator ─> /explore/resume
     (idle/pending/following)                  (true=探索 / false=暂停)
```

#### 安装 explore_lite（本仓库不内置）

搜索入口先启动编排节点，再延迟 2 秒创建 explore_lite，确保默认暂停状态先到达；融合状态超过 3 秒未更新时按无目标处理。`min_frontier_size` 单位为米，默认 0.5。

```bash
cd ~/mini_car_ws/src
git clone https://github.com/robo-friends/m-explore-ros2.git
cd ~/mini_car_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select explore_lite --symlink-install --parallel-workers 1
```

启动：

```bash
ros2 launch rescue_perception rescue_search.launch.py
```

> **自主搜索会让机器人自行移动，属高风险行为，默认不自动启动。**
> 确认场地清空、急停可用后再开启：
>
> ```bash
> ros2 topic pub --once /rescue/search_cmd std_msgs/msg/Bool "{data: true}"
> ```
>
> 停止：把 `data` 置 `false`。

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `auto_start` | `false` | 上电即搜索（高风险，务必确认场地清空） |
| `resume_delay` | `5.0` | 目标结束后延迟多少秒恢复探索，避免启停抖动 |
| `start_explore` | `true` | 是否启动 `explore_lite`；false 时只启动编排节点 |
| `enable` | `true` | false 时编排节点忽略一切外部指令 |

状态机：`IDLE`（不发指令）→ `SEARCHING`（探索）→ `YIELDED`（发现目标，让位）
→ 目标结束并等待 `resume_delay` 后回到 `SEARCHING`。当前状态发布在
`/rescue/search_state`。

探索开关按 1Hz 持续下发而非只在变化时发一次——`explore_lite` 可能晚于编排
节点启动，去重会导致它错过初始的暂停指令而上电即探索。

至此「搜索 → 检测 → 规划 → 逼近」完整闭环已打通。

## 6. 速度指令仲裁

Nav2、KCF 跟随、Web 遥操都会下发速度指令，未经仲裁会互相打架。`slam_navigation.launch.py` 默认启用 `twist_mux` 仲裁：

```text
Nav2        /cmd_vel         ┐
KCF 跟随    /cmd_vel_kcf     ├─> twist_mux ─> /cmd_vel_muxed ─> 底盘
Web 遥操    /cmd_vel_teleop  ┘
```

优先级（`config/twist_mux.yaml`）：Web 遥操 100 > KCF 跟随 50 > Nav2 10，另有一路 `255` 的急停锁。每个源独立 `timeout`，超时未收到消息即视为失效并自动降级；全部失效时输出零速度——**发布方节点崩溃时底盘会自动停车**。

首次运行需安装：

```bash
sudo apt install ros-humble-twist-mux
```

关闭仲裁（回退到旧行为，底盘直接监听 Nav2 的 `/cmd_vel`）：

```bash
ros2 launch turn_on_wheeltec_robot slam_navigation.launch.py use_twist_mux:=false
```

> 急停锁只是软件层屏蔽，不能替代硬件急停开关。

## 7. 系统架构

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

## 8. ROS2 迁移说明

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

## 9. STM32 固件

### 9.1 基本信息

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

### 9.2 车型对应

| STM32 枚举 | ROS2 model | 车型 |
| --- | --- | --- |
| Mec_Car | mini_mec | 麦克纳姆 |
| Omni_Car | mini_omni | 三轮全向 |
| Akm_Car | mini_akm | 阿克曼 |
| Diff_Car | mini_diff | 两轮差速 |
| FourWheel_Car | mini_4wd | 四轮驱动 |

STM32 电位器档位必须与 ROS2 的 `model` 参数一致。

### 9.3 安全提示

STM32 原固件没有通信超时停车机制。ROS2 桥接已增加上位机看门狗，但**仍建议在 STM32 侧增加独立通信看门狗**——树莓派死机时 ROS2 无法继续发送零速度。

## 10. 串口协议

> STM32 通信协议保持不变，因此已烧录的霍尔编码器版固件可以继续使用。

默认设备 `/dev/wheeltec_controller`，115200 bit/s，8N1，对应 STM32 USART3。

### 10.1 ROS2 到 STM32

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

### 10.2 STM32 到 ROS2

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

## 11. ROS2 话题与 TF

| 名称 | 类型 | 方向 |
| --- | --- | --- |
| `/cmd_vel` | geometry_msgs/msg/Twist | Nav2 -> twist_mux（仲裁输入，优先级 10） |
| `/cmd_vel_kcf` | geometry_msgs/msg/Twist | KCF 跟随 -> twist_mux（优先级 50） |
| `/cmd_vel_teleop` | geometry_msgs/msg/Twist | Web 网关 -> twist_mux（优先级 100） |
| `/cmd_vel_muxed` | geometry_msgs/msg/Twist | twist_mux -> 底盘（唯一发给底盘的指令） |
| `/cmd_vel_estop_lock` | std_msgs/msg/Bool | 急停锁，置 true 屏蔽全部速度源 |
| `/follow_target` | kcf_track/action/FollowTarget | 两阶段融合跟随动作 |
| `/kcf/track` | geometry_msgs/msg/Twist | KCF 跟踪输出：`linear.x`=距离、`angular.z`=像素横坐标 |
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

### 11.1 深度相机在建图中的角色

`slam_navigation.launch.py` 中 RTAB-Map 同时订阅 RGB-D 与激光（`subscribe_rgbd: true`、`subscribe_scan: true`）：

- **RGB-D**（Astra Pro）负责视觉特征、回环检测、图优化约束与 3D 重建；
- **2D 激光**（A1M8）负责生成 2D 占用栅格 `/map`（`Grid/FromDepth: false`），供 Nav2 做 2D 代价地图与规划。

两者都在参与 SLAM，只是分工不同。厂房为平面环境、Nav2 为 2D 导航，因此 2D 栅格由激光生成，比深度图投影更稳定。

## 12. Astra Pro 深度相机驱动

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

## 13. udev 串口配置

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

## 14. RPLIDAR A1M8 接入

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

## 15. 救援场景传感器建议

### 15.1 基础闭环

| 传感器 | 作用 | 当前工程状态 |
| --- | --- | --- |
| 霍尔编码器 | 短时轮速与里程计 | STM32 已有 |
| 6 轴 IMU | 角速度、加速度和短时姿态约束 | STM32 已有，当前回传原始量 |
| 2D 激光雷达 | 平面避障和激光 SLAM | A1M8，`/scan`，驱动已内置 |
| RGB-D 相机 | 视觉回环、深度障碍和目标跟踪 | Astra Pro，launch 已接入 |

### 15.2 厂房救援推荐加固

普通 RGB-D 相机不应作为救援环境唯一的定位或避障传感器。烟尘、黑暗、强逆光、反光金属、热源和水雾都会使深度图或视觉特征退化。建议按预算增加：

- 3D 激光雷达：优先级最高，提供对烟尘和弱光更稳定的几何信息；RTAB-Map 可接入 3D 点云或 2D 投影。
- 热成像相机：搜索人员和高温区域，发布独立检测结果，不直接替代导航定位。
- 气体传感器：CO、CO2、VOC、氧气和可燃气体，用于风险评估与返航策略。
- UWB 定位基站或无线测距：厂房遮挡、重复纹理或烟尘导致 SLAM 退化时提供全局约束。
- 独立急停、碰撞条和安全遥控链路：不依赖 ROS2 进程，硬件层切断电机使能。
- 电池、电流和温度监测：支持低电量返航、过流保护和热失控预警。

如果只能增加一种定位相关传感器，优先选择带 IMU 的 3D 激光雷达；如果任务重点是找人，增加热成像相机和气体传感器，但仍保留激光雷达作为避障主传感器。

### 15.3 救援系统安全边界

该系统适合人在回路的实验和辅助侦察，不应直接视为消防或生命安全认证设备。必须提供人工接管、急停、失联停车、低电量停车、传感器失效降级和通信日志；正式部署前应在烟雾、弱光、反光、狭窄通道和动态障碍物条件下做分级测试。

## 16. 常见问题

### 16.1 串口打不开

```bash
ls -l /dev/wheeltec_controller
groups
sudo usermod -aG dialout $USER
```

加入 `dialout` 后需重新登录。**不要使用 `sudo ros2 launch` 绕过设备权限。**

### 16.2 有 /cmd_vel 但小车不动

- 等待 STM32 完成约 10 秒 IMU 校准。
- 检查电池是否高于 10 V。
- 检查硬件使能开关与 `/chassis_enabled`。
- 检查 ROS2 `model` 与 STM32 电位器档位。

### 16.3 RTAB-Map 没有输出 /map

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /camera/color/camera_info
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_tools view_frames
```

RGB、深度和相机内参时间戳无法同步时，`rgbd_sync` 不会输出有效数据。

### 16.4 Nav2 报 map 或 TF 超时

检查 `map -> odom -> base_footprint -> camera_link` 是否完整。若相机驱动已经发布 base 到 camera 的 TF，可关闭本工程的相机静态 TF：

```bash
publish_camera_tf:=false
```

### 16.5 编译失败

```bash
cd ~/mini_car_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --event-handlers console_direct+
```

常见原因是相机第三方库（magic_enum、libuvc）未安装，见第 2.3 节。

### 16.6 Web 控制台页面空白或无画面

- 服务未起来：确认启动时已 `source` ROS2 与工作空间，缺 `rclpy` 会直接报错退出。
- 画面黑屏：查 `http://<目标机IP>:8000/api/status` 的 `video` 字段，`error` 会说明是相机未启动还是图像编码不支持（当前支持 `rgb8`/`bgr8`）。
- 地图空白：确认 RTAB-Map 已在发布 `/map`。

## 17. Git 工作流

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

## 18. 当前限制

仓库提供 `python -m unittest discover -s tests -v` 的纯逻辑回归测试，覆盖 KCF 控制律和
控制台地图 RLE；涉及 ROS2、相机、串口和实车运动的测试仍必须在 Ubuntu 22.04/Humble
目标机按分层清单执行。

- 开发机（Windows）无 ROS2 Humble，只能做静态语法与结构验证；`colcon build` 与实车行为必须在 Ubuntu 22.04 / Humble 上验证。
- Web 控制台桥接层固定依赖 `rclpy`，无法在无 ROS2 环境运行期自检，全部运行时验证需在目标机完成。
- `twist_mux` 仲裁已实装但**尚未在目标机验证**（需先 `apt install ros-humble-twist-mux`）。
- 两阶段融合跟随（`FollowTarget`）已实装但**尚未实车验证**：`colcon build` 需生成 action 接口，staging 与伺服两阶段的实际衔接效果待验证。
- 自主目标检测（`rescue_perception`）已实装，核心逻辑已在 conda 环境验证（ultralytics API、检测框字段、深度换算/采样/反投影、稳定性与置信度分级，均通过）；但**未在目标机实车验证**——深度单位（Astra 默认 16UC1 毫米）与真实场景检测精度需实测确认。
- 自主搜索（`search_coordinator`）状态机已用 conda 环境验证通过，但**依赖的 `m-explore-ros2` 未内置仓库**，需按 5.8 节自行 clone 构建；实车探索行为（frontier 选择、与接近动作的切换）待验证。
- 完整链路「搜索 → 检测 → 规划 → 逼近」已打通代码路径，但**每一环都还缺实车验证**：深度单位、检测精度、仲裁生效、两阶段衔接均未实测。
- STM32 侧仍建议增加独立通信看门狗（树莓派死机时 ROS2 无法发零速度）。
- Web 控制台实时画面当前仅支持 `rgb8`/`bgr8` 未压缩编码；若相机发布 `mjpeg` 等压缩格式需改用 `cv_bridge`。
- Nav2 参数主要针对 Mini 麦克纳姆底盘，其他车型需要实车调参。
- ROS1 多点导航脚本没有迁移，不参与 ROS2 安装。
- 视频回传目前只有彩色画面，深度图伪彩未实现。

## 19. 参考资料

- [RTAB-Map](https://github.com/introlab/rtabmap)
- [rtabmap_ros](https://github.com/introlab/rtabmap_ros)
- [RTAB-Map 安装说明](https://github.com/introlab/rtabmap/wiki/Installation)
- [ROS2 Humble 文档](https://docs.ros.org/en/humble/)
- [Nav2 文档](https://docs.nav2.org/)
- [项目 Gitee 仓库](https://gitee.com/qbz23/mini_car_project)
- [Web 救援控制台说明](rescue_console/README.md)
