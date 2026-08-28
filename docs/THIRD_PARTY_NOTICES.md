# 第三方组件来源与许可说明

本仓库包含为实现完整 ROS2 机器人工作空间所需的第三方驱动源码。升级、再发布或替换这些目录前，必须确认上游许可和二进制分发权限。

## RPLIDAR ROS2 驱动

- 路径：src/rplidar_ros
- 上游：https://github.com/Slamtec/rplidar_ros
- 使用分支：ros2
- 导入提交：24cc9b6，提交说明为修复 nullptr_t 编译问题。
- 许可：BSD，原始 LICENSE 文件已保留。
- 对应硬件：SLAMTEC RPLIDAR A1M8。

## Orbbec Astra 驱动

- 路径：src/astra_camera、src/astra_camera_msgs
- 来源：随车资料 humble-src-2023-12-29.zip 内的 ros2_astra_camera。
- 对应硬件：Orbbec Astra Pro，深度设备 USB 2bc5:0403，彩色 UVC 设备 USB 2bc5:0502。
- 组成：ROS2 驱动源码、OpenNI2 头文件、arm/arm64/x64 的 OpenNI2 与 Orbbec 二进制库。
- 许可状态：上游源码头部声明 Orbbec 3D Technology 的专有权利，随车包 package.xml 未提供独立许可证文本。

不要将 Astra 目录替换为另一版本而不验证 OpenNI2 ABI、aarch64 二进制兼容性和供应商分发许可。若必须公开发布镜像或二进制，请先核对采购/随车许可。

## 修改规则

- 不删除任何第三方 LICENSE、README、NOTICE 或驱动二进制。
- 对第三方代码做补丁时，单独提交并写明上游来源与修改目的。
- 不将第三方目录再初始化为嵌套 Git 仓库。
- 不在工作空间内保留第二份同名驱动源码，否则 colcon 会报 Duplicate package names。
