# ROS1 迁移参考资料

本目录集中存放项目早期 ROS1 版本的 launch、参数、示例地图和辅助脚本，便于迁移排查与历史对照。

目录内容不属于当前 ROS2 工作空间的 `src/`，没有 `package.xml`，不会被 `colcon` 构建或安装。当前运行入口、ROS2 参数和 ROS2 Python launch 请查看 `src/` 下对应包。

其中的 XML launch 和 Python 脚本依赖 ROS1 的 `roscore`、`rospy`、`roscpp`、`move_base` 等接口，不能直接用于 ROS2 Humble。若需要引用其中的参数或算法，应先迁移并在 ROS2 包中建立经过验证的新配置。

## 内容

- `turn_on_wheeltec_robot/launch/`：旧底盘、建图、导航、传感器和规划 XML launch。
- `turn_on_wheeltec_robot/params_*`：旧 `move_base`、DWA、TEB 和 costmap 参数。
- `turn_on_wheeltec_robot/map/`：旧静态地图及 ROS1 `map_saver` launch。
- `turn_on_wheeltec_robot/scripts/send_mark.py`：ROS1 多点导航辅助脚本。
- `kcf_track/`：旧 KCF XML launch 与 ROS1 图像缩放脚本。
