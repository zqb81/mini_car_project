cmake_minimum_required(VERSION 3.8)
project(kcf_track)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -O3)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 两阶段融合跟随（follow_target）的动作接口。动作定义放本包内，
# 因为它是 KCF 跟随能力的对外契约，不通用到需要单独建 msg 包。
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FollowTarget.action"
  DEPENDENCIES geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(kcf_node
  src/fhog.cpp
  src/kcftracker.cpp
  src/runtracker.cpp
)
target_include_directories(kcf_node PRIVATE src ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(kcf_node
  cv_bridge
  geometry_msgs
  rclcpp
  sensor_msgs
)
target_link_libraries(kcf_node ${OpenCV_LIBRARIES})

install(TARGETS kcf_node
  DESTINATION lib/${PROJECT_NAME}
)
# kcf_control.py 是被另两个脚本 import 的控制律模块，必须与它们同目录
install(PROGRAMS
  scripts/kcf_follow.py
  scripts/kcf_control.py
  scripts/follow_target_server.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

ament_package()
