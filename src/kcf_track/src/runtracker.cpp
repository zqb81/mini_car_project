#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/twist.hpp"
#include "kcftracker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"

namespace
{

constexpr char kRgbWindow[] = "KCF RGB";

}  // namespace

class KcfTrackerNode final : public rclcpp::Node
{
public:
  KcfTrackerNode()
  : Node("kcf_tracker"), tracker_(true, false, true, false)
  {
    rgb_topic_ = declare_parameter<std::string>(
      "rgb_topic", "/camera/color/image_raw");
    depth_topic_ = declare_parameter<std::string>(
      "depth_topic", "/camera/depth/image_raw");
    show_window_ = declare_parameter<bool>("show_window", true);

    const int roi_x = declare_parameter<int>("initial_roi.x", 0);
    const int roi_y = declare_parameter<int>("initial_roi.y", 0);
    const int roi_width = declare_parameter<int>("initial_roi.width", 0);
    const int roi_height = declare_parameter<int>("initial_roi.height", 0);
    if (roi_width > 0 && roi_height > 0) {
      selected_roi_ = cv::Rect(roi_x, roi_y, roi_width, roi_height);
      renew_roi_ = true;
    }

    tracking_publisher_ = create_publisher<geometry_msgs::msg::Twist>("kcf/track", 10);
    rgb_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      rgb_topic_, rclcpp::SensorDataQoS(),
      std::bind(&KcfTrackerNode::onRgbImage, this, std::placeholders::_1));
    depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&KcfTrackerNode::onDepthImage, this, std::placeholders::_1));
    roi_subscription_ = create_subscription<sensor_msgs::msg::RegionOfInterest>(
      "/rescue/target_roi", 10,
      std::bind(&KcfTrackerNode::onRoi, this, std::placeholders::_1));

    if (show_window_) {
      cv::namedWindow(kRgbWindow);
      cv::setMouseCallback(kRgbWindow, &KcfTrackerNode::mouseCallback, this);
    }
  }

  ~KcfTrackerNode() override
  {
    if (show_window_) {
      cv::destroyWindow(kRgbWindow);
    }
  }

private:
  void onRoi(const sensor_msgs::msg::RegionOfInterest::ConstSharedPtr message)
  {
    // 仅接受非空框；下一帧 RGB 到达时统一裁剪并初始化，避免跨线程修改跟踪器。
    if (message->width > 4 && message->height > 4) {
      selected_roi_ = cv::Rect(
        static_cast<int>(message->x_offset), static_cast<int>(message->y_offset),
        static_cast<int>(message->width), static_cast<int>(message->height));
      renew_roi_ = true;
    }
  }

  static void mouseCallback(int event, int x, int y, int, void * context)
  {
    static_cast<KcfTrackerNode *>(context)->handleMouse(event, x, y);
  }

  void handleMouse(int event, int x, int y)
  {
    if (selecting_) {
      selected_roi_.x = std::min(selection_origin_.x, x);
      selected_roi_.y = std::min(selection_origin_.y, y);
      selected_roi_.width = std::abs(x - selection_origin_.x);
      selected_roi_.height = std::abs(y - selection_origin_.y);
      if (!rgb_image_.empty()) {
        selected_roi_ &= cv::Rect(0, 0, rgb_image_.cols, rgb_image_.rows);
      }
    }

    if (event == cv::EVENT_LBUTTONDOWN) {
      tracking_active_ = false;
      selecting_ = true;
      selection_origin_ = cv::Point(x, y);
      selected_roi_ = cv::Rect(x, y, 0, 0);
    } else if (event == cv::EVENT_LBUTTONUP) {
      selecting_ = false;
      renew_roi_ = selected_roi_.width > 4 && selected_roi_.height > 4;
    }
  }

  void onRgbImage(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    try {
      rgb_image_ = cv_bridge::toCvCopy(
        message, sensor_msgs::image_encodings::BGR8)->image;
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR(get_logger(), "RGB 图像转换失败：%s", error.what());
      return;
    }

    const cv::Rect image_bounds(0, 0, rgb_image_.cols, rgb_image_.rows);
    if (renew_roi_) {
      selected_roi_ &= image_bounds;
      if (selected_roi_.width > 4 && selected_roi_.height > 4) {
        tracker_.init(selected_roi_, rgb_image_);
        tracking_active_ = true;
      }
      renew_roi_ = false;
    }

    if (tracking_active_) {
      tracked_roi_ = tracker_.update(rgb_image_) & image_bounds;
      if (tracked_roi_.width <= 4 || tracked_roi_.height <= 4) {
        tracking_active_ = false;
      } else {
        cv::rectangle(rgb_image_, tracked_roi_, cv::Scalar(0, 255, 255), 2);
      }
    } else if (selected_roi_.width > 0 && selected_roi_.height > 0) {
      cv::rectangle(rgb_image_, selected_roi_, cv::Scalar(255, 0, 0), 2);
    }

    if (show_window_) {
      cv::imshow(kRgbWindow, rgb_image_);
      cv::waitKey(1);
    }
  }

  void onDepthImage(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    if (!tracking_active_ || tracked_roi_.width <= 4 || tracked_roi_.height <= 4) {
      return;
    }

    try {
      const auto depth = cv_bridge::toCvShare(message);
      if (rgb_image_.empty()) {
        return;
      }

      const double scale_x =
        static_cast<double>(depth->image.cols) / static_cast<double>(rgb_image_.cols);
      const double scale_y =
        static_cast<double>(depth->image.rows) / static_cast<double>(rgb_image_.rows);
      const cv::Rect bounds(0, 0, depth->image.cols, depth->image.rows);
      const cv::Rect roi(
        static_cast<int>(std::lround(tracked_roi_.x * scale_x)),
        static_cast<int>(std::lround(tracked_roi_.y * scale_y)),
        static_cast<int>(std::lround(tracked_roi_.width * scale_x)),
        static_cast<int>(std::lround(tracked_roi_.height * scale_y)));
      const cv::Rect depth_roi = roi & bounds;
      if (depth_roi.width <= 4 || depth_roi.height <= 4) {
        return;
      }

      const std::array<cv::Point, 5> points = {
        cv::Point(depth_roi.x + depth_roi.width / 3, depth_roi.y + depth_roi.height / 3),
        cv::Point(
          depth_roi.x + 2 * depth_roi.width / 3,
          depth_roi.y + depth_roi.height / 3),
        cv::Point(
          depth_roi.x + depth_roi.width / 3,
          depth_roi.y + 2 * depth_roi.height / 3),
        cv::Point(
          depth_roi.x + 2 * depth_roi.width / 3,
          depth_roi.y + 2 * depth_roi.height / 3),
        cv::Point(
          depth_roi.x + depth_roi.width / 2,
          depth_roi.y + depth_roi.height / 2),
      };

      std::vector<double> valid_depths;
      valid_depths.reserve(points.size());
      for (const auto & point : points) {
        double distance = 0.0;
        if (message->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
          distance = depth->image.at<float>(point);
        } else if (
          message->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
          message->encoding == sensor_msgs::image_encodings::MONO16)
        {
          distance = depth->image.at<uint16_t>(point) / 1000.0;
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "不支持深度图编码 %s，仅支持 32FC1 和 16UC1。",
            message->encoding.c_str());
          return;
        }

        if (std::isfinite(distance) && distance >= 0.2 && distance <= 10.0) {
          valid_depths.push_back(distance);
        }
      }

      geometry_msgs::msg::Twist tracking;
      if (!valid_depths.empty()) {
        const double sum =
          std::accumulate(valid_depths.begin(), valid_depths.end(), 0.0);
        tracking.linear.x = sum / static_cast<double>(valid_depths.size());
        tracking.angular.y = tracked_roi_.y + tracked_roi_.height * 0.5;
        tracking.angular.z = tracked_roi_.x + tracked_roi_.width * 0.5;
      }
      tracking_publisher_->publish(tracking);
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR(get_logger(), "深度图像转换失败：%s", error.what());
    }
  }

  std::string rgb_topic_;
  std::string depth_topic_;
  bool show_window_{true};
  bool selecting_{false};
  bool renew_roi_{false};
  bool tracking_active_{false};
  cv::Point selection_origin_;
  cv::Rect selected_roi_;
  cv::Rect tracked_roi_;
  cv::Mat rgb_image_;
  KCFTracker tracker_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr tracking_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::RegionOfInterest>::SharedPtr roi_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KcfTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
