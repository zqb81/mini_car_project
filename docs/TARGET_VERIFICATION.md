# 目标机验证清单

本清单用于首次在目标机（Ubuntu 22.04 + ROS2 Humble）上验证本工程。

**按分层顺序执行，不要跳步**——上层依赖下层，跳步会让故障定位变得困难
（例如 SLAM 没输出时，很难区分是相机问题还是 TF 问题）。

每一层都给出：命令、预期结果、失败排查。**未通过就不要进入下一层。**

> **安全**：凡涉及电机动作的验证，**必须先让车轮悬空**，并确认硬件使能
> 开关与急停随手可关。

---

## 0. 环境准备

### 0.1 构建

```bash
sudo apt update
sudo apt install -y ros-humble-twist-mux

git clone git@gitee.com:qbz23/mini_car_project.git ~/mini_car_ws
cd ~/mini_car_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

**预期**：`colcon build` 无错误退出。`rescue_perception` 会随其他包一起构建
（纯 Python，无编译）；`kcf_track` 会生成 `FollowTarget` action 接口。

**排查**：
- 内存不足（树莓派常见）→ 按包分步构建，见 README 第 4 节
- 缺 magic_enum / libuvc → 见 README 第 2.3 节
- `FollowTarget` 接口未生成 → `ros2 interface show kcf_track/action/FollowTarget`

### 0.2 接口自检（不需硬件）

```bash
ros2 interface show kcf_track/action/FollowTarget
ros2 pkg list | grep -E "rescue_perception|kcf_track|twist_mux"
ros2 pkg executables rescue_perception
```

**预期**：
- action 接口能显示，含 `use_staging_pose`、`staging_pose`、`target_distance`、`servo_timeout`
- 三个包都在列表中
- `rescue_perception` 有三个可执行：`detect_target`、`target_fusion`、`search_coordinator`

### 0.3 设备路径

```bash
ls -l /dev/wheeltec_controller /dev/wheeltec_lidar
groups | grep dialout
```

**预期**：两个软链接存在且当前用户属于 `dialout`。

**排查**：不在 dialout 组则 `sudo usermod -aG dialout $USER` 并**重新登录**。
不要用 `sudo ros2 launch` 绕过权限。

---

## 1. 底盘

```bash
ros2 launch turn_on_wheeltec_robot base.launch.py \
  model:=mini_mec serial_port:=/dev/wheeltec_controller
```

另开终端：

```bash
ros2 topic echo --once /PowerVoltage
ros2 topic echo --once /odom
ros2 topic echo --once /chassis_enabled
ros2 run tf2_ros tf2_echo odom base_footprint
```

**预期**：
- `/PowerVoltage` 约 11–13 V（低于 10 V 应充电）
- `/odom` 持续输出
- `tf2_echo` 能输出变换

**排查**：
- 串口打不开 → 见 README 第 16.1 节
- 无任何输出 → 检查 STM32 电位器档位与 `model` 参数是否一致

### 1.1 电机点动（**车轮必须悬空**）

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**预期**：车轮慢速转动；**停止发布后 0.5 秒内自动停车**（上位机看门狗）。

**排查**：
- 不动 → 等 STM32 完成约 10 秒 IMU 校准；检查电池 >10 V、使能开关、
  `/chassis_enabled`、`model` 与电位器档位

---

## 2. 雷达

```bash
ros2 launch turn_on_wheeltec_robot rplidar_a1.launch.py \
  serial_port:=/dev/wheeltec_lidar
ros2 topic hz /scan
```

**预期**：`/scan` 稳定在约 5–10 Hz。

**排查**：无数据 → 检查串口权限与波特率（115200）、`scan_mode:=Standard`

---

## 3. 相机

```bash
ros2 launch turn_on_wheeltec_robot astra_pro.launch.py
```

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo --once /camera/color/camera_info
```

**预期**：三个话题都有输出；`camera_info` 的 `k[0]`（fx）非 0。

**排查**：
- 无图像 → 检查 USB（`2bc5:0403` 深度、`2bc5:0502` 彩色）
- `camera_info` 内参全 0 → 相机未正确标定，检测的深度投影会失效

### 3.1 确认深度单位（**关键，影响测距精度**）

```bash
ros2 topic echo --once /camera/depth/image_raw --field encoding
```

**预期**：通常为 `16UC1`（单位毫米）。若是 `32FC1` 则单位为米。

`detect_target` 已按编码自动换算（16UC1 ÷1000，32FC1 原样）。若出现其他
编码，节点会告警并跳过深度处理——此时需确认实际单位后再扩展
`_DEPTH_SCALE`。

**实测校验**：把已知距离的物体（如 1.5 m 处的墙）放在画面中心，运行检测后
对比 `/detect_target/detections_3d` 中的距离值。偏差超过 10% 说明单位或
对齐有问题。

---

## 4. SLAM

```bash
ros2 launch turn_on_wheeltec_robot slam_navigation.launch.py \
  model:=mini_mec \
  serial_port:=/dev/wheeltec_controller \
  database_path:=$HOME/.ros/mini_car_slam.db
```

```bash
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_footprint
```

**预期**：`/map` 有输出；`map -> odom` 与 `map -> base_footprint` 都能查到。

**排查**：
- 无 `/map` → 依次确认 `/camera/color/image_raw`、`/camera/depth/image_raw`、
  `/camera/color/camera_info`、`/scan`、`/odom` 都有数据；RGB 与深度时间戳
  不同步时 `rgbd_sync` 不会输出
- TF 超时 → 见 README 第 16.4 节

---

## 5. Nav2

承上，SLAM 入口已包含 Nav2。

```bash
ros2 node list | grep -E "controller_server|planner_server|bt_navigator"
ros2 action list | grep navigate_to_pose
```

**预期**：三个生命周期节点存在，`navigate_to_pose` 动作可用。

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
```

**预期**：均为 `active`（`autostart` 默认 true）。

### 5.1 下发导航目标（**车轮悬空或场地清空**）

用 RViz 的 "Nav2 Goal"，或命令行：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map},
           pose: {position: {x: 1.0, y: 0.0, z: 0.0},
                  orientation: {w: 1.0}}}}"
```

**预期**：机器人朝目标移动并避障。

---

## 6. 速度指令仲裁（twist_mux）

```bash
ros2 node list | grep twist_mux
ros2 topic list | grep cmd_vel
```

**预期**：存在 `/cmd_vel`、`/cmd_vel_kcf`、`/cmd_vel_teleop`、`/cmd_vel_muxed`、
`/cmd_vel_estop_lock`。

### 6.1 验证底盘只听仲裁输出

```bash
ros2 topic info /cmd_vel_muxed
```

**预期**：`twist_mux` 为发布者，底盘节点为订阅者。

### 6.2 验证优先级与超时

1. 发布导航目标（Nav2 占用 `/cmd_vel`）
2. 手动向 `/cmd_vel_teleop` 发一条速度：

```bash
ros2 topic pub --rate 10 /cmd_vel_teleop geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**预期**：遥操（优先级 100）压过导航（10）。

3. **停止发布遥操**，等待 1 秒

**预期**：超时后自动降级回导航——`/cmd_vel_muxed` 不再输出遥操的速度。

### 6.3 验证急停锁

```bash
ros2 topic pub --once /cmd_vel_estop_lock std_msgs/msg/Bool "{data: true}"
```

**预期**：`/cmd_vel_muxed` 输出零速度（所有源被屏蔽）。

解锁：`data: false`。

> 这是软件层屏蔽，**不能替代硬件急停**。

---

## 7. Web 救援控制台

```bash
cd ~/mini_car_ws
python3 -m venv rescue_console/venv
./rescue_console/venv/bin/pip install -r rescue_console/server/requirements.txt

source /opt/ros/humble/setup.bash
source ~/mini_car_ws/install/setup.bash
cd ~/mini_car_ws/rescue_console/server
./../../venv/bin/python -m uvicorn app:app --host 0.0.0.0 --port 8000
```

浏览器打开 `http://<目标机IP>:8000`。

```bash
curl http://localhost:8000/api/status
```

**预期**：JSON 中 `cmd_vel_topic` 为 `/cmd_vel_teleop`；`video.has_frame` 为
`true` 且 `error` 为 `null`。

**排查**：
- `video.error` 提示编码不支持（非 `rgb8`/`bgr8`）→ 需改用 `cv_bridge`
- 遥控无效 → 核对 `cmd_vel_topic`：若关闭了仲裁需设 `RESCUE_CMD_VEL_TOPIC=/cmd_vel`

---

## 8. 两阶段融合跟随（FollowTarget）

```bash
ros2 launch kcf_track kcf_tracking.launch.py \
  follow_mode:=fusion \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw
```

```bash
ros2 action list | grep follow_target
```

**预期**：`follow_target` 动作可用。

在 KCF 窗口框选目标（无显示器时先按 README 第 5.6 节指定 `initial_roi`），
然后：

```bash
ros2 action send_goal /follow_target kcf_track/action/FollowTarget \
  "{use_staging_pose: false, target_distance: 1.2, servo_timeout: 30.0}"
```

**预期**：机器人向目标逼近并保持约 1.2 m；反馈中 `phase` 从 0 变为 2。

### 8.1 验证目标丢失停车

遮挡相机或让目标离开视野。

**预期**：约 0.5 秒（`tracking_timeout`）内停车，动作返回
`error_code=3`（TARGET_LOST）。

### 8.2 验证两阶段（需建图已完成）

```bash
ros2 action send_goal /follow_target kcf_track/action/FollowTarget \
  "{use_staging_pose: true,
    staging_pose: {header: {frame_id: map},
                   pose: {position: {x: 2.0, y: 1.0, z: 0.0},
                          orientation: {w: 1.0}}},
    target_distance: 1.2,
    servo_timeout: 60.0}"
```

**预期**：阶段 1 由 Nav2 导航到 staging 点（享有避障），阶段 2 切到视觉伺服。
反馈中 `phase` 依次为 1 → 2。

**排查**：
- `error_code=1`（NAV2_UNAVAILABLE）→ Nav2 未启动或 bt_navigator 非 active
- `error_code=2`（STAGING_FAILED）→ 目标点不可达，换一个位置
- `error_code=3`（TARGET_LOST）→ 目标不在视野或深度无效

---

## 9. 自主目标检测

```bash
pip3 install -r ~/mini_car_ws/src/rescue_perception/requirements.txt
ros2 launch rescue_perception detect_target.launch.py target_classes:=person
```

```bash
ros2 topic echo /rescue/target_pose
ros2 run rqt_image_view rqt_image_view   # 选 /detect_target/debug_image
```

**预期**：有人在画面中时，`/rescue/target_pose` 输出 map 系位姿；
调试图上画出检测框与测距值。

**排查**：
- 无输出 → 检查 `/camera/color/image_raw` 有数据、TF 链路完整
- 距离明显错误（差 1000 倍）→ 深度单位问题，见第 3.1 节
- 距离偏大/偏小 → 检查 `depth_align` 是否开启（见 README 第 5.7 节）

### 9.1 记录实际帧率

```bash
ros2 topic hz /detect_target/detections_3d
```

**预期**：受 `min_interval` 限流，默认约 2 Hz。若远低于此，说明算力不足，
考虑降分辨率或加加速器。

---

## 10. 检测与导航融合

```bash
ros2 launch rescue_perception rescue_perception.launch.py auto_mode:=false
```

**先用 `auto_mode:=false` 验证人工确认链路**（更安全）：

```bash
ros2 topic echo /rescue/pending_target
```

**预期**：检测到目标后输出待确认位姿，且**不会自动移动**。

```bash
ros2 topic pub --once /rescue/confirm std_msgs/msg/Bool "{data: true}"
```

**预期**：机器人开始向目标移动。

确认链路正常后，再测自动模式：

```bash
ros2 launch rescue_perception rescue_perception.launch.py auto_mode:=true
```

**预期**：高置信（≥0.75）且连续稳定 3 次后自动下发导航。

> 阈值（`auto_conf_threshold` / `confirm_conf_threshold`）需结合实际场景调整；
> 若误检频繁，调高阈值或关闭自动模式。

---

## 11. 自主搜索

先安装 explore_lite（本仓库不内置）：

```bash
cd ~/mini_car_ws/src
git clone https://github.com/robo-friends/m-explore-ros2.git
cd ~/mini_car_ws
colcon build --packages-select explore_lite --symlink-install --parallel-workers 1
```

```bash
ros2 launch rescue_perception rescue_search.launch.py
ros2 topic echo /rescue/search_state
```

**预期**：初始为 `idle`，探索被暂停。

**开启搜索（确认场地清空、急停可用）**：

```bash
ros2 topic pub --once /rescue/search_cmd std_msgs/msg/Bool "{data: true}"
```

**预期**：
- `/rescue/search_state` 变为 `searching`，机器人开始探索未知区域
- 检测到目标后变为 `yielded`，探索停止，转由 FollowTarget 接近
- 目标结束并等待 `resume_delay`（默认 5 s）后恢复 `searching`

**停止**：

```bash
ros2 topic pub --once /rescue/search_cmd std_msgs/msg/Bool "{data: false}"
```

**排查**：
- 状态一直 `idle` → 检查 `/rescue/search_cmd` 话题名、编排节点是否运行
- 开启后机器人不动 → 检查 `explore_lite` 是否运行、`/map` 是否有未知区域
- 检测到目标但未让位 → 检查 `/rescue/fusion_state` 是否有值
  （需 `rescue_perception.launch.py` 在运行）

---

## 验证结果记录

建议把每层结果记录下来，便于后续追溯与调参：

| 层 | 通过 | 实测值 / 备注 |
| --- | --- | --- |
| 0 环境准备 | ☐ | colcon build 结果、接口自检 |
| 1 底盘 | ☐ | 电池电压、看门狗停车时间 |
| 2 雷达 | ☐ | /scan 频率 |
| 3 相机 | ☐ | **深度编码、实测距离误差** |
| 4 SLAM | ☐ | /map 频率、TF 完整性 |
| 5 Nav2 | ☐ | 生命周期状态、导航效果 |
| 6 仲裁 | ☐ | 优先级切换、超时降级、急停锁 |
| 7 Web 控制台 | ☐ | 画面帧率、遥控响应 |
| 8 融合跟随 | ☐ | 逼近距离、丢失停车、两阶段切换 |
| 9 目标检测 | ☐ | **检测精度、实际帧率** |
| 10 融合决策 | ☐ | 阈值是否合适、误检次数 |
| 11 自主搜索 | ☐ | frontier 选择、让位切换 |

**特别关注第 3 层的深度单位与第 9 层的检测精度**——这两项是开发机上无法
验证、只能靠实测确认的假设。
