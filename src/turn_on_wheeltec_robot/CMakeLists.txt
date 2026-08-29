cmake_minimum_required(VERSION 3.8)
project(turn_on_wheeltec_robot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(wheeltec_robot_node
  src/wheeltec_robot.cpp
)
target_include_directories(wheeltec_robot_node PRIVATE include)
ament_target_dependencies(wheeltec_robot_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
)

install(TARGETS wheeltec_robot_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config urdf
  DESTINATION share/${PROJECT_NAME}
)

# 仅安装 ROS2 Python launch；ROS1 XML 资产已移至仓库根目录 ros1_rsc/，不参与安装。
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

install(PROGRAMS scripts/wheeltec_udev.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
