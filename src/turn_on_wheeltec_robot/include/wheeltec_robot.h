#ifndef TURN_ON_WHEELTEC_ROBOT__WHEELTEC_ROBOT_H_
#define TURN_ON_WHEELTEC_ROBOT__WHEELTEC_ROBOT_H_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace turn_on_wheeltec_robot
{

class WheeltecRobotNode final : public rclcpp::Node
{
public:
  // 初始化 ROS2 话题、串口连接及安全定时器；串口暂不可用时由定时器负责重连。
  WheeltecRobotNode();
  // 析构前主动下发零速度，避免正常退出 ROS2 进程时底盘保持上一帧运动指令。
  ~WheeltecRobotNode() override;

private:
  // STM32 二进制协议的固定帧边界与帧长；命令帧和状态帧不可混用。
  static constexpr uint8_t kFrameHeader = 0x7B;
  static constexpr uint8_t kFrameTail = 0x7D;
  static constexpr std::size_t kCommandFrameSize = 11;
  static constexpr std::size_t kStatusFrameSize = 24;
  static constexpr double kAccelerometerRatio = 1671.84;
  static constexpr double kGyroscopeRatio = 0.00026644;

  // 接收 Nav2 或遥控的速度指令，限幅后编码为 STM32 可表示的 int16 数据。
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr message);
  // 非阻塞读取串口并累积字节流，读取错误后关闭描述符以触发后续自动重连。
  void readSerial();
  // 上位机速度指令超时后仅发送一次零速度；超时值小于等于零时显式关闭该保护。
  void checkCommandTimeout();
  // 从可能错位或半包的字节流中提取完整且校验通过的状态帧。
  void processReceiveBuffer();
  // 将状态帧转换为里程计、IMU、电池电压与底盘使能状态，并按需发布 odom TF。
  void handleStatusFrame(const std::array<uint8_t, kStatusFrameSize> & frame);

  // 配置 8N1 原始串口；失败时记录下一次允许重连的时间点。
  bool openSerial();
  void closeSerial();
  // 按协议将 m/s、rad/s 速度缩放为千分单位；数值超出 int16 范围时饱和截断。
  bool writeCommand(double velocity_x, double velocity_y, double velocity_z);
  bool writeAll(const uint8_t * data, std::size_t size);

  static uint8_t checksum(const uint8_t * data, std::size_t size);
  static int16_t decodeInt16(uint8_t high, uint8_t low);
  static void encodeInt16(int16_t value, uint8_t & high, uint8_t & low);
  static geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);

  std::string serial_port_;
  int baud_rate_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_frame_;
  bool publish_odom_tf_;
  double command_timeout_seconds_;
  double max_linear_speed_;
  double max_lateral_speed_;
  double max_angular_speed_;

  int serial_fd_{-1};
  std::vector<uint8_t> receive_buffer_;
  std::chrono::steady_clock::time_point next_reconnect_attempt_;
  rclcpp::Time last_command_time_;
  rclcpp::Time last_odom_time_;
  bool timeout_stop_sent_{false};
  double position_x_{0.0};
  double position_y_{0.0};
  double heading_{0.0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr chassis_enabled_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr serial_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace turn_on_wheeltec_robot

#endif  // TURN_ON_WHEELTEC_ROBOT__WHEELTEC_ROBOT_H_
