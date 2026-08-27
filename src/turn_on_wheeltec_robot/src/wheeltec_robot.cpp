#include "wheeltec_robot.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <functional>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/transform_stamped.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

speed_t baudToTermios(int baud_rate)
{
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    default:
      return B115200;
  }
}

double clampValue(double value, double limit)
{
  return std::clamp(value, -std::abs(limit), std::abs(limit));
}

}  // namespace

namespace turn_on_wheeltec_robot
{

WheeltecRobotNode::WheeltecRobotNode()
: Node("wheeltec_robot")
{
  serial_port_ = declare_parameter<std::string>("serial_port", "/dev/wheeltec_controller");
  baud_rate_ = declare_parameter<int>("baud_rate", 115200);
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
  imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link");
  publish_odom_tf_ = declare_parameter<bool>("publish_odom_tf", true);
  command_timeout_seconds_ = declare_parameter<double>("cmd_vel_timeout", 0.5);
  max_linear_speed_ = declare_parameter<double>("max_linear_speed", 1.5);
  max_lateral_speed_ = declare_parameter<double>("max_lateral_speed", 1.5);
  max_angular_speed_ = declare_parameter<double>("max_angular_speed", 3.0);

  if (baud_rate_ != 9600 && baud_rate_ != 19200 && baud_rate_ != 38400 &&
    baud_rate_ != 57600 && baud_rate_ != 115200)
  {
    RCLCPP_WARN(
      get_logger(), "不支持波特率 %d，串口将按 115200 配置。", baud_rate_);
    baud_rate_ = 115200;
  }

  odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", 20);
  imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
    "imu", rclcpp::SensorDataQoS());
  voltage_publisher_ = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 10);
  chassis_enabled_publisher_ =
    create_publisher<std_msgs::msg::Bool>("chassis_enabled", 10);

  cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&WheeltecRobotNode::onCmdVel, this, std::placeholders::_1));

  if (publish_odom_tf_) {
    odom_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  receive_buffer_.reserve(1024);
  last_command_time_ = now();
  last_odom_time_ = now();
  next_reconnect_attempt_ = std::chrono::steady_clock::now();

  openSerial();

  serial_timer_ = create_wall_timer(
    std::chrono::milliseconds(5), std::bind(&WheeltecRobotNode::readSerial, this));
  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&WheeltecRobotNode::checkCommandTimeout, this));
}

WheeltecRobotNode::~WheeltecRobotNode()
{
  if (serial_fd_ >= 0) {
    writeCommand(0.0, 0.0, 0.0);
  }
  closeSerial();
}

void WheeltecRobotNode::onCmdVel(const geometry_msgs::msg::Twist::SharedPtr message)
{
  const double velocity_x = clampValue(message->linear.x, max_linear_speed_);
  const double velocity_y = clampValue(message->linear.y, max_lateral_speed_);
  const double velocity_z = clampValue(message->angular.z, max_angular_speed_);

  if (writeCommand(velocity_x, velocity_y, velocity_z)) {
    last_command_time_ = now();
    timeout_stop_sent_ = false;
  }
}

void WheeltecRobotNode::readSerial()
{
  if (serial_fd_ < 0) {
    if (std::chrono::steady_clock::now() >= next_reconnect_attempt_) {
      openSerial();
    }
    return;
  }

  std::array<uint8_t, 256> chunk{};
  while (true) {
    const ssize_t bytes_read = ::read(serial_fd_, chunk.data(), chunk.size());
    if (bytes_read > 0) {
      receive_buffer_.insert(
        receive_buffer_.end(), chunk.begin(), chunk.begin() + bytes_read);
      continue;
    }

    if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_ERROR(
        get_logger(), "读取串口 %s 失败：%s", serial_port_.c_str(), std::strerror(errno));
      closeSerial();
    }
    break;
  }

  if (receive_buffer_.size() > 4096) {
    RCLCPP_WARN(get_logger(), "串口接收缓存异常增长，丢弃旧数据并重新同步帧头。");
    receive_buffer_.erase(receive_buffer_.begin(), receive_buffer_.end() - 1024);
  }
  processReceiveBuffer();
}

void WheeltecRobotNode::checkCommandTimeout()
{
  if (timeout_stop_sent_ || command_timeout_seconds_ <= 0.0) {
    return;
  }

  if ((now() - last_command_time_).seconds() >= command_timeout_seconds_) {
    if (writeCommand(0.0, 0.0, 0.0)) {
      timeout_stop_sent_ = true;
      RCLCPP_WARN(
        get_logger(), "超过 %.3f 秒未收到 cmd_vel，已向 STM32 发送零速度。",
        command_timeout_seconds_);
    }
  }
}

void WheeltecRobotNode::processReceiveBuffer()
{
  while (receive_buffer_.size() >= kStatusFrameSize) {
    const auto header = std::find(
      receive_buffer_.begin(), receive_buffer_.end(), kFrameHeader);
    if (header == receive_buffer_.end()) {
      receive_buffer_.clear();
      return;
    }
    if (header != receive_buffer_.begin()) {
      receive_buffer_.erase(receive_buffer_.begin(), header);
    }
    if (receive_buffer_.size() < kStatusFrameSize) {
      return;
    }

    if (receive_buffer_[kStatusFrameSize - 1] != kFrameTail ||
      checksum(receive_buffer_.data(), 22) != receive_buffer_[22])
    {
      receive_buffer_.erase(receive_buffer_.begin());
      continue;
    }

    std::array<uint8_t, kStatusFrameSize> frame{};
    std::copy_n(receive_buffer_.begin(), kStatusFrameSize, frame.begin());
    receive_buffer_.erase(
      receive_buffer_.begin(), receive_buffer_.begin() + kStatusFrameSize);
    handleStatusFrame(frame);
  }
}

void WheeltecRobotNode::handleStatusFrame(
  const std::array<uint8_t, kStatusFrameSize> & frame)
{
  const double velocity_x = decodeInt16(frame[2], frame[3]) / 1000.0;
  const double velocity_y = decodeInt16(frame[4], frame[5]) / 1000.0;
  const double velocity_z = decodeInt16(frame[6], frame[7]) / 1000.0;
  const auto stamp = now();

  double delta_time = (stamp - last_odom_time_).seconds();
  last_odom_time_ = stamp;
  if (delta_time < 0.0 || delta_time > 0.5) {
    delta_time = 0.0;
  }

  position_x_ +=
    (velocity_x * std::cos(heading_) - velocity_y * std::sin(heading_)) * delta_time;
  position_y_ +=
    (velocity_x * std::sin(heading_) + velocity_y * std::cos(heading_)) * delta_time;
  heading_ = std::remainder(heading_ + velocity_z * delta_time, 2.0 * kPi);
  const auto orientation = yawToQuaternion(heading_);

  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = odom_frame_;
  odometry.child_frame_id = base_frame_;
  odometry.pose.pose.position.x = position_x_;
  odometry.pose.pose.position.y = position_y_;
  odometry.pose.pose.orientation = orientation;
  odometry.twist.twist.linear.x = velocity_x;
  odometry.twist.twist.linear.y = velocity_y;
  odometry.twist.twist.angular.z = velocity_z;
  odometry.pose.covariance[0] = 0.01;
  odometry.pose.covariance[7] = 0.01;
  odometry.pose.covariance[35] = 0.05;
  odometry.twist.covariance[0] = 0.02;
  odometry.twist.covariance[7] = 0.02;
  odometry.twist.covariance[35] = 0.05;
  odom_publisher_->publish(odometry);

  if (odom_tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = position_x_;
    transform.transform.translation.y = position_y_;
    transform.transform.rotation = orientation;
    odom_tf_broadcaster_->sendTransform(transform);
  }

  sensor_msgs::msg::Imu imu;
  imu.header.stamp = stamp;
  imu.header.frame_id = imu_frame_;
  imu.orientation.w = 1.0;
  // STM32 状态帧不包含可靠的绝对姿态，因此按 ROS 约定标记姿态不可用。
  imu.orientation_covariance[0] = -1.0;
  imu.linear_acceleration.x = decodeInt16(frame[8], frame[9]) / kAccelerometerRatio;
  imu.linear_acceleration.y = decodeInt16(frame[10], frame[11]) / kAccelerometerRatio;
  imu.linear_acceleration.z = decodeInt16(frame[12], frame[13]) / kAccelerometerRatio;
  imu.angular_velocity.x = decodeInt16(frame[14], frame[15]) * kGyroscopeRatio;
  imu.angular_velocity.y = decodeInt16(frame[16], frame[17]) * kGyroscopeRatio;
  imu.angular_velocity.z = decodeInt16(frame[18], frame[19]) * kGyroscopeRatio;
  imu.angular_velocity_covariance[0] = 0.02;
  imu.angular_velocity_covariance[4] = 0.02;
  imu.angular_velocity_covariance[8] = 0.02;
  imu.linear_acceleration_covariance[0] = 0.1;
  imu.linear_acceleration_covariance[4] = 0.1;
  imu.linear_acceleration_covariance[8] = 0.1;
  imu_publisher_->publish(imu);

  std_msgs::msg::Float32 voltage;
  voltage.data = decodeInt16(frame[20], frame[21]) / 1000.0F;
  voltage_publisher_->publish(voltage);

  std_msgs::msg::Bool chassis_enabled;
  chassis_enabled.data = frame[1] == 0;
  chassis_enabled_publisher_->publish(chassis_enabled);
}

bool WheeltecRobotNode::openSerial()
{
  next_reconnect_attempt_ =
    std::chrono::steady_clock::now() + std::chrono::seconds(2);
  closeSerial();

  serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    RCLCPP_WARN(
      get_logger(), "无法打开串口 %s：%s，2 秒后重试。",
      serial_port_.c_str(), std::strerror(errno));
    return false;
  }

  termios settings{};
  if (tcgetattr(serial_fd_, &settings) != 0) {
    RCLCPP_ERROR(get_logger(), "读取串口配置失败：%s", std::strerror(errno));
    closeSerial();
    return false;
  }

  cfmakeraw(&settings);
  const speed_t speed = baudToTermios(baud_rate_);
  cfsetispeed(&settings, speed);
  cfsetospeed(&settings, speed);
  settings.c_cflag |= CLOCAL | CREAD;
  settings.c_cflag &= ~CSTOPB;
  settings.c_cflag &= ~CRTSCTS;
  settings.c_cflag &= ~PARENB;
  settings.c_cflag &= ~CSIZE;
  settings.c_cflag |= CS8;
  settings.c_cc[VMIN] = 0;
  settings.c_cc[VTIME] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &settings) != 0) {
    RCLCPP_ERROR(get_logger(), "写入串口配置失败：%s", std::strerror(errno));
    closeSerial();
    return false;
  }
  tcflush(serial_fd_, TCIOFLUSH);
  receive_buffer_.clear();
  RCLCPP_INFO(
    get_logger(), "已打开 STM32 串口 %s，波特率 %d。", serial_port_.c_str(), baud_rate_);
  return true;
}

void WheeltecRobotNode::closeSerial()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool WheeltecRobotNode::writeCommand(
  double velocity_x, double velocity_y, double velocity_z)
{
  if (serial_fd_ < 0) {
    if (std::chrono::steady_clock::now() < next_reconnect_attempt_ || !openSerial()) {
      return false;
    }
  }

  const auto to_protocol_value = [](double value) {
      const long scaled = std::lround(value * 1000.0);
      return static_cast<int16_t>(std::clamp(scaled, -32768L, 32767L));
    };

  std::array<uint8_t, kCommandFrameSize> frame{};
  frame[0] = kFrameHeader;
  encodeInt16(to_protocol_value(velocity_x), frame[3], frame[4]);
  encodeInt16(to_protocol_value(velocity_y), frame[5], frame[6]);
  encodeInt16(to_protocol_value(velocity_z), frame[7], frame[8]);
  frame[9] = checksum(frame.data(), 9);
  frame[10] = kFrameTail;
  return writeAll(frame.data(), frame.size());
}

bool WheeltecRobotNode::writeAll(const uint8_t * data, std::size_t size)
{
  std::size_t written = 0;
  while (written < size) {
    const ssize_t result = ::write(serial_fd_, data + written, size - written);
    if (result > 0) {
      written += static_cast<std::size_t>(result);
      continue;
    }

    if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      pollfd descriptor{serial_fd_, POLLOUT, 0};
      if (::poll(&descriptor, 1, 20) > 0) {
        continue;
      }
    }

    RCLCPP_ERROR(
      get_logger(), "写入串口 %s 失败：%s", serial_port_.c_str(), std::strerror(errno));
    closeSerial();
    return false;
  }
  return true;
}

uint8_t WheeltecRobotNode::checksum(const uint8_t * data, std::size_t size)
{
  uint8_t result = 0;
  for (std::size_t index = 0; index < size; ++index) {
    result ^= data[index];
  }
  return result;
}

int16_t WheeltecRobotNode::decodeInt16(uint8_t high, uint8_t low)
{
  return static_cast<int16_t>(
    (static_cast<uint16_t>(high) << 8U) | static_cast<uint16_t>(low));
}

void WheeltecRobotNode::encodeInt16(int16_t value, uint8_t & high, uint8_t & low)
{
  const uint16_t encoded = static_cast<uint16_t>(value);
  high = static_cast<uint8_t>((encoded >> 8U) & 0xFFU);
  low = static_cast<uint8_t>(encoded & 0xFFU);
}

geometry_msgs::msg::Quaternion WheeltecRobotNode::yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

}  // namespace turn_on_wheeltec_robot

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turn_on_wheeltec_robot::WheeltecRobotNode>());
  rclcpp::shutdown();
  return 0;
}
