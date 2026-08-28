# 救援控制台（rescue_console）

厂房救援场景的 Web 客户端：小车侧 FastAPI 网关 + 浏览器控制页。
客户端**不依赖 ROS**，通过 HTTP + WebSocket 与网关通信；网关桥接层可在
「模拟数据源」与「ROS2 真实话题」之间切换。

## 1. 目录结构

~~~text
rescue_console/
├── server/
│   ├── app.py           FastAPI 网关：路由、WebSocket 广播、10Hz 主循环
│   ├── bridge.py        桥接层：MockBridge（模拟）/ RosBridge（rclpy）
│   └── requirements.txt Web 依赖（版本范围兼容 Python 3.10+）
├── web/
│   └── index.html       控制页：实时地图、激光、轨迹、遥控、导航目标
└── deploy/
    ├── install.sh                  可选：注册 systemd 开机自启（venv + 服务）
    ├── run_mock.sh                 模拟模式自检（不依赖 ROS、不注册服务）
    └── rescue-console.service.template  systemd 服务模板
~~~

## 2. 运行（Demo 模式，不需要 ROS2）

~~~bash
cd rescue_console/server
python -m pip install -r requirements.txt
python -m uvicorn app:app --host 0.0.0.0 --port 8000
~~~

浏览器打开 `http://<主机IP>:8000`。

Demo 模式使用 MockBridge：合成厂房环境（20m x 16m，含立柱、隔墙、货架），
激光射线仿真对真值地图步进，同时生成「探索区域逐渐扩大」的实时建图效果，
模拟救援场景"无法提前建图、边走边建"的核心需求。

## 3. 页面功能

| 功能 | 操作 | 说明 |
| --- | --- | --- |
| 实时地图 | 自动 | 占用栅格 + 激光扫描 + 行驶轨迹，未知区域不渲染 |
| 手动遥控 | W/S/A/D/Q/E 或按住屏幕按钮 | 前后/转向/平移，10Hz 心跳发送 |
| 紧急停车 | 「停」按钮 | 取消导航并立即发送零速度 |
| 导航目标 | 点击地图 | Demo 为演示级 P 控制器；实车由 Nav2 规划 |
| 状态面板 | 自动 | 电池（低电量告警）、位姿、速度、里程、底盘使能 |

## 4. 网关协议

### 4.1 HTTP

| 方法 | 路径 | 请求体 | 说明 |
| --- | --- | --- | --- |
| GET | /api/status | - | 桥接模式与客户端数 |
| POST | /api/cmd_vel | {vx, vy, wz} | 米/秒、弧度/秒；0.5s 无新指令自动停车 |
| POST | /api/nav_goal | {x, y} | 地图坐标（米） |
| POST | /api/cancel_nav | - | 取消导航并停车 |

### 4.2 WebSocket `/ws/telemetry`

连接后立即补发一帧遥测与当前地图，之后服务端推送：

~~~json
{"type": "telemetry", "pose": {"x":0,"y":0,"theta":0}, "twist": {...},
 "battery": 12.6, "battery_low": false, "chassis_enabled": true,
 "odometry": 0.0, "nav_state": "idle", "nav_goal": null, "manual": false,
 "scan": {"angle_min": 3.14, "angle_max": -3.14, "range_max": 12.0,
          "ranges": [/* -1 表示超量程 */]}}
~~~

~~~json
{"type": "map", "width": 400, "height": 320, "resolution": 0.05,
 "origin": [0, 0], "rle": [[count, value], ...]}
~~~

地图为 RLE 行程编码的 OccupancyGrid 语义（-1 未知 / 0 自由 / 100 占用），
仅在栅格更新时约 1Hz 推送，单帧通常几 KB。

## 5. 目标机部署（树莓派 + Ubuntu 22.04 + ROS2 Humble）

本项目以**源码**方式交付：目标机 `git pull` 拉源码后直接运行，
不需要打包、不需要离线 wheel。

### 5.1 源码运行（推荐，三步）

~~~bash
# 1. 拉取源码（目标机标准布局：仓库根即 colcon 工作空间）
cd ~/mini_car_ws
git pull --ff-only

# 2. 装 Web 依赖到 venv（rclpy 不进 venv，由系统 ROS 提供，互不污染）
python3 -m venv rescue_console/venv
./rescue_console/venv/bin/pip install -r rescue_console/server/requirements.txt

# 3. 运行（ros 模式接真实话题；先用 --mock 变量自检）
source /opt/ros/humble/setup.bash
source ~/mini_car_ws/install/setup.bash
cd ~/mini_car_ws/rescue_console/server
RESCUE_BRIDGE=ros ./../../venv/bin/python -m uvicorn app:app --host 0.0.0.0 --port 8000
~~~

浏览器访问 `http://<目标机IP>:8000`。停止服务：Ctrl+C。

### 5.2 部署链路自检（先跑这个，再接实车）

~~~bash
cd ~/mini_car_ws/rescue_console/deploy
./run_mock.sh
~~~

模拟模式不依赖 ROS、不注册服务。自检通过标志：浏览器能看到地图、
遥控小车移动、地图随探索扩大。**先自检再接实车**。

### 5.3 开机自启（可选）

~~~bash
cd ~/mini_car_ws/rescue_console/deploy
./install.sh            # 部署为 ros 模式（接真实 ROS2 话题）
./install.sh --mock     # 或部署为模拟模式（链路自检用）
~~~

脚本自动完成：安装 `python3-venv`（apt）；创建 venv 并安装 Web 依赖；
由模板生成 `/etc/systemd/system/rescue-console.service` 并启用开机自启
（服务启动命令内先 source ROS 与工作空间环境，再启动 venv 中的 uvicorn）。

~~~bash
systemctl status rescue-console        # 状态
journalctl -u rescue-console -f        # 日志
sudo systemctl edit rescue-console     # 修改 RESCUE_BRIDGE 等环境变量
~~~

### 5.4 ros 模式运行前提

1. `slam_navigation.launch.py` 已在运行（底盘 + 雷达 + 相机 + RTAB-Map + Nav2）。
2. Nav2 三个生命周期节点为 active（controller/planner/bt_navigator）。
3. `/cmd_vel` 由本网关与 Nav2 共用——**当前无 twist_mux 仲裁**，手动遥控
   与自动导航不要同时进行（网关在收到手动指令时会先取消 Nav2 目标，但
   不能替代仲裁器；正式使用前应部署 twist_mux）。
4. 激光安装朝向偏移默认 π（随车默认值，README 16.2），可用环境变量
   `RESCUE_LASER_YAW_OFFSET` 覆盖，**实车地图与激光方向不一致时首先核对它**。

RosBridge 数据通路（与 docs/AGENT_HANDOFF.md 第 7 节话题契约一致）：
订阅 `/odom` `/PowerVoltage` `/chassis_enabled` `/scan` `/map`（transient_local
QoS，兼容 latched 地图）与 TF；发布 `/cmd_vel`；导航目标走 Nav2
`NavigateToPose` action，支持到达/失败/取消结果回传。

### 5.5 开发机与目标机的兼容性差异

| 差异点 | 开发机 | 目标机 | 影响 |
| --- | --- | --- | --- |
| 操作系统 | Windows 11 | Ubuntu 22.04 (aarch64) | 代码全部走 pathlib/标准库，无 Windows 专有路径 |
| Python | 3.13 | 3.10 | 依赖版本范围已同时覆盖两者；代码未用 3.11+ 语法 |
| 依赖 wheel | x86-64 | aarch64 | fastapi/pydantic-core/uvicorn 在 aarch64 均有官方 wheel，纯 pip 安装即可 |
| ROS2 | 无 | Humble | mock 模式完全隔离；ros 模式仅在目标机可运行（缺 rclpy 时报中文错误提示） |
| 文件编码 | GBK 控制台 | UTF-8 | 源文件均带 UTF-8 声明；uvicorn 日志两侧均正常 |

注意：开发机已安装的 fastapi 0.141 超出 requirements.txt 上限（<0.116 是为
兼容目标机 Python 3.10 收紧的），**目标机以 requirements.txt 为准**，两者
协议行为一致。

## 6. 安全约束（与 AGENTS.md 一致）

- 手动遥控带 0.5s 看门狗：浏览器关闭、断网、卡顿时小车自动停车
  （网关与 wheeltec_robot_node 双层看门狗）。
- 任何手动输入会取消自动导航（人工接管优先）。
- **接实车前必须车轮悬空或场地清空**，且确认硬件急停可用。
- KCF 与 Nav2 与本客户端三方都发布 /cmd_vel，接实车前必须加 twist_mux 仲裁。
- 服务绑定 0.0.0.0 且无鉴权，仅限可信局域网使用；跨网段暴露需自加反向代理与认证。
- 视频回传未实现，页面为占位块；后续可加 MJPEG（/camera/color/image_raw）。

## 7. 已验证 / 待验证

| 项目 | 状态 |
| --- | --- |
| Demo 模式 REST/WS 接口 | 已在本机验证（Windows + Python 3.13） |
| 5Hz 遥测推送、RLE 地图推送、首连补发 | 已验证 |
| 遥控联动、看门狗停车、导航目标、取消导航 | 已验证（模拟环境） |
| RosBridge（rclpy 实装） | 代码已完成，**待目标机实车验证**（按 AGENTS.md 分层验收） |
| 源码运行流程（git pull + venv + uvicorn） | 待目标机验证 |
| deploy/install.sh（可选 systemd） | 语法已检查，未实际执行 |
| NavigateToPose action 交互、TF 位姿、激光偏移 | 待目标机验证 |
| 视频回传、twist_mux | 未实现，后续迭代 |
