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
install(PROGRAMS scripts/kcf_follow.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

ament_package()
