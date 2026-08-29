# 救援控制台（rescue_console）

厂房救援场景的 Web 客户端：小车侧 FastAPI 网关 + 浏览器控制页。
客户端**不依赖 ROS**，通过 HTTP + WebSocket 与网关通信；网关桥接层
（RosBridge）通过 rclpy 直接读写目标机上的真实 ROS2 话题与 Nav2 动作。

**运行前提**：必须在 source 过 ROS2 Humble 与 colcon 工作空间的环境里启动，
即目标机（Ubuntu 22.04 + ROS2 Humble）。无 ROS2 的环境无法启动本服务。

## 1. 目录结构

~~~text
rescue_console/
├── server/
│   ├── app.py           FastAPI 网关：路由、WebSocket 广播、10Hz 主循环
│   ├── bridge.py        桥接层：BaseBridge 接口 + RosBridge（rclpy）
│   └── requirements.txt Web 依赖（版本范围兼容 Python 3.10+）
├── web/
│   └── index.html       控制页：实时地图、激光、轨迹、遥控、导航目标
└── deploy/
    ├── install.sh       可选：注册 systemd 开机自启（venv + 服务）
    └── rescue-console.service.template  systemd 服务模板
~~~

## 2. 运行

~~~bash
# 必须先 source ROS2 与工作空间，否则 rclpy 导入失败（报中文错误提示）
source /opt/ros/humble/setup.bash
source ~/mini_car_ws/install/setup.bash

cd ~/mini_car_ws/rescue_console/server
~/mini_car_ws/rescue_console/venv/bin/python -m uvicorn app:app \
    --host 0.0.0.0 --port 8000
~~~

浏览器打开 `http://<目标机IP>:8000`。停止服务：Ctrl+C。

地图由 RTAB-Map 实时构建并通过 `/map` 话题下发，控制台只做转发与渲染，
因此天然满足救援场景"无法提前建图、边走边建"的核心需求。

## 3. 页面功能

| 功能 | 操作 | 说明 |
| --- | --- | --- |
| 实时地图 | 自动 | 占用栅格 + 激光扫描 + 行驶轨迹，未知区域不渲染 |
| 手动遥控 | W/S/A/D/Q/E 或按住屏幕按钮 | 前后/转向/平移，10Hz 心跳发送 |
| 紧急停车 | 「停」按钮 | 取消导航并立即发送零速度 |
| 导航目标 | 点击地图 | 由 Nav2 规划路径并驱动底盘 |
| 实时画面 | 自动 | MJPEG 播放 `/camera/color/image_raw`，限流 10fps |
| 状态面板 | 自动 | 电池（低电量告警）、位姿、速度、里程、底盘使能 |

## 4. 网关协议

### 4.1 HTTP

| 方法 | 路径 | 请求体 | 说明 |
| --- | --- | --- | --- |
| GET | /api/status | - | 桥接模式与客户端数 |
| POST | /api/cmd_vel | {vx, vy, wz} | 米/秒、弧度/秒；0.5s 无新指令自动停车 |
| POST | /api/nav_goal | {x, y} | 地图坐标（米） |
| POST | /api/cancel_nav | - | 取消导航并停车 |
| POST | /api/estop | {locked:true/false} | 锁定或解除软件急停；解锁不会自动恢复运动 |
| GET | /video/stream | - | MJPEG 彩色画面（`multipart/x-mixed-replace`），前端 `<img src>` 直接播放 |

`/api/status` 额外返回 `video` 字段（`topic` / `fps` / `encoding` / `has_frame` /
`error`），用于在目标机排查"画面黑屏"是相机没启动、话题名不对，还是编码不支持。

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

# 3. 运行（先 source ROS 与工作空间环境）
source /opt/ros/humble/setup.bash
source ~/mini_car_ws/install/setup.bash
cd ~/mini_car_ws/rescue_console/server
./../../venv/bin/python -m uvicorn app:app --host 0.0.0.0 --port 8000
~~~

浏览器访问 `http://<目标机IP>:8000`。停止服务：Ctrl+C。

### 5.2 开机自启（可选）

~~~bash
cd ~/mini_car_ws/rescue_console/deploy
./install.sh
~~~

脚本自动完成：安装 `python3-venv`（apt）；创建 venv 并安装 Web 依赖；
由模板生成 `/etc/systemd/system/rescue-console.service` 并启用开机自启
（服务启动命令内先 source ROS 与工作空间环境，再启动 venv 中的 uvicorn）。

~~~bash
systemctl status rescue-console        # 状态
journalctl -u rescue-console -f        # 日志
sudo systemctl edit rescue-console     # 修改 RESCUE_LASER_YAW_OFFSET 等环境变量
~~~

生产或跨网段使用前，建议创建 `~/.config/rescue-console.env` 并写入
`RESCUE_API_TOKEN=一段随机长令牌`，然后重启服务。浏览器首次访问时可使用
`http://<目标机IP>:8000/#token=一段随机长令牌`，令牌会保存到浏览器本地存储；
不要把令牌放进普通查询参数或提交到仓库。

### 5.3 运行前提（接实车前逐条确认）

1. `slam_navigation.launch.py` 已在运行（底盘 + 雷达 + 相机 + RTAB-Map + Nav2）。
2. Nav2 三个生命周期节点为 active（controller/planner/bt_navigator）。
3. `slam_navigation.launch.py` 默认启用 `twist_mux` 仲裁，底盘只接收
   `/cmd_vel_muxed`。因此本网关**默认发布 `/cmd_vel_teleop`**（而非直发
   `/cmd_vel`）。若关闭了仲裁（`use_twist_mux:=false`），必须把网关的输出
   改回 `/cmd_vel`，否则遥控无效：

   ```bash
   RESCUE_CMD_VEL_TOPIC=/cmd_vel ./../../venv/bin/python -m uvicorn app:app ...
   ```

   `/api/status` 的 `cmd_vel_topic` 字段会返回当前实际发布话题，可用于
   排查“发了指令但小车不动”。
4. 激光安装朝向偏移默认 π（随车默认值，README 16.2），可用环境变量
   `RESCUE_LASER_YAW_OFFSET` 覆盖，**实车地图与激光方向不一致时首先核对它**。

RosBridge 数据通路（与 docs/AGENT_HANDOFF.md 第 7 节话题契约一致）：
订阅 `/odom` `/PowerVoltage` `/chassis_enabled` `/scan` `/map`（transient_local
QoS，兼容 latched 地图）与 TF；发布速度指令（默认 `/cmd_vel_teleop`，由
`twist_mux` 按优先级 100 仲裁，高于 Nav2 的 10 与 KCF 跟随的 50）；
导航目标走 Nav2 `NavigateToPose` action，支持到达/失败/取消结果回传。

### 5.4 开发机与目标机的兼容性差异

| 差异点 | 开发机 | 目标机 | 影响 |
| --- | --- | --- | --- |
| 操作系统 | Windows 11 | Ubuntu 22.04 (aarch64) | 代码全部走 pathlib/标准库，无 Windows 专有路径 |
| Python | 3.13 | 3.10 | 依赖版本范围已同时覆盖两者；代码未用 3.11+ 语法 |
| 依赖 wheel | x86-64 | aarch64 | fastapi/pydantic-core/uvicorn 在 aarch64 均有官方 wheel，纯 pip 安装即可 |
| ROS2 | 无 | Humble | **本服务只能在目标机运行**；缺 rclpy 时启动即报中文错误提示 |
| 文件编码 | GBK 控制台 | UTF-8 | 源文件均带 UTF-8 声明；uvicorn 日志两侧均正常 |

注意：开发机已安装的 fastapi 0.141 超出 requirements.txt 上限（<0.116 是为
兼容目标机 Python 3.10 收紧的），**目标机以 requirements.txt 为准**，两者
协议行为一致。

> 桥接层此前提供过不依赖 ROS 的模拟实现（MockBridge），用于开发机演示与
> 链路自检；现已移除，仓库只保留面向目标机的 RosBridge 实现。

## 6. 安全约束（与 AGENTS.md 一致）

- 手动遥控带 0.5s 看门狗：浏览器关闭、断网、卡顿时小车自动停车
  （网关与 wheeltec_robot_node 双层看门狗）。
- 任何手动输入会取消自动导航（人工接管优先）。
- **接实车前必须车轮悬空或场地清空**，且确认硬件急停可用。
- 本网关、Nav2、KCF 三方速度指令由 `twist_mux` 仲裁（遥操 100 > 跟随 50 >
  导航 10）；关闭仲裁时不得让任意两路同时下发。`twist_mux` 只是软件层仲裁，
  不能替代硬件急停。
- 服务绑定 0.0.0.0 且无鉴权，仅限可信局域网使用；跨网段暴露需自加反向代理与认证。
- 可通过 `RESCUE_API_TOKEN` 启用控制接口 Bearer 鉴权；浏览器访问时把令牌放入 URL 片段
  `#token=...`，页面会保存到本地存储。未配置令牌仅适合隔离的开发网络。
- 「急停」按钮会锁定 `/cmd_vel_estop_lock`、取消网关导航并发送零速；须点击「解除软件急停锁」
  后重新发送速度。该锁仍是软件保护，不能替代硬件急停。
- 浏览器窗口失焦、页面隐藏或 WebSocket 断开时会清空遥控按键并发送零速。
- 实时画面仅支持 rgb8 / bgr8 未压缩编码（Pillow 解码）。若相机发布 mjpeg
  等压缩格式，页面会显示明确的编码错误，此时改用 cv_bridge：
  `apt install ros-humble-cv-bridge`（需同步改造 bridge.py 的 _encode_jpeg）。

## 7. 已验证 / 待验证

| 项目 | 状态 |
| --- | --- |
| Python 语法（bridge.py / app.py） | 已验证（ast.parse） |
| `_encode_jpeg` 参数传递与错误分支 | 已验证（桩对象单测：rgb8/bgr8 映射、`step` 作 stride、不支持编码的提示） |
| 实际 JPEG 编码与画面显示 | **待目标机验证**（开发机无法访问 PyPI，未装 Pillow） |
| 5Hz 遥测推送、RLE 地图推送、首连补发 | 代码已完成，待目标机验证 |
| MJPEG 多客户端共享单份编码、10fps 限流 | 代码已完成，待目标机验证 |
| RosBridge（rclpy 实装） | 代码已完成，**待目标机实车验证**（按 AGENTS.md 分层验收） |
| 源码运行流程（git pull + venv + uvicorn） | 待目标机验证 |
| deploy/install.sh（可选 systemd） | 语法已检查，未实际执行 |
| NavigateToPose action 交互、TF 位姿、激光偏移 | 待目标机验证 |
| twist_mux 仲裁 | 已接入，待目标机验证（配置显式使用 `Twist`） |

> 说明：由于桥接层固定依赖 rclpy，**本服务无法在开发机（Windows，无 ROS2）
> 上做任何运行期自检**，全部运行时验证需在目标机完成。
