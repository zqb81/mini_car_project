/*
Author: Christian Bailer
Contact address: Christian.Bailer@dfki.de
Department Augmented Vision DFKI

3-clause BSD License
*/

#pragma once

#include <cassert>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace RectTools
{

template<typename T>
inline cv::Vec<T, 2> center(const cv::Rect_<T> & rectangle)
{
  return cv::Vec<T, 2>(
    rectangle.x + rectangle.width / static_cast<T>(2),
    rectangle.y + rectangle.height / static_cast<T>(2));
}

template<typename T>
inline T x2(const cv::Rect_<T> & rectangle)
{
  return rectangle.x + rectangle.width;
}

template<typename T>
inline T y2(const cv::Rect_<T> & rectangle)
{
  return rectangle.y + rectangle.height;
}

template<typename T>
inline void resize(cv::Rect_<T> & rectangle, float scale_x, float scale_y = 0.0F)
{
  if (scale_y == 0.0F) {
    scale_y = scale_x;
  }
  rectangle.x -= rectangle.width * (scale_x - 1.0F) / 2.0F;
  rectangle.width *= scale_x;
  rectangle.y -= rectangle.height * (scale_y - 1.0F) / 2.0F;
  rectangle.height *= scale_y;
}

template<typename T>
inline void limit(cv::Rect_<T> & rectangle, const cv::Rect_<T> & bounds)
{
  if (rectangle.x + rectangle.width > bounds.x + bounds.width) {
    rectangle.width = bounds.x + bounds.width - rectangle.x;
  }
  if (rectangle.y + rectangle.height > bounds.y + bounds.height) {
    rectangle.height = bounds.y + bounds.height - rectangle.y;
  }
  if (rectangle.x < bounds.x) {
    rectangle.width -= bounds.x - rectangle.x;
    rectangle.x = bounds.x;
  }
  if (rectangle.y < bounds.y) {
    rectangle.height -= bounds.y - rectangle.y;
    rectangle.y = bounds.y;
  }
  rectangle.width = std::max<T>(rectangle.width, 0);
  rectangle.height = std::max<T>(rectangle.height, 0);
}

template<typename T>
inline void limit(cv::Rect_<T> & rectangle, T width, T height, T x = 0, T y = 0)
{
  limit(rectangle, cv::Rect_<T>(x, y, width, height));
}

template<typename T>
inline cv::Rect getBorder(const cv::Rect_<T> & original, const cv::Rect_<T> & limited)
{
  cv::Rect_<T> result;
  result.x = limited.x - original.x;
  result.y = limited.y - original.y;
  result.width = x2(original) - x2(limited);
  result.height = y2(original) - y2(limited);
  assert(result.x >= 0 && result.y >= 0 && result.width >= 0 && result.height >= 0);
  return result;
}

inline cv::Mat subwindow(
  const cv::Mat & input, const cv::Rect & window,
  int border_type = cv::BORDER_CONSTANT)
{
  cv::Rect cut_window = window;
  limit(cut_window, input.cols, input.rows);
  assert(cut_window.height > 0 && cut_window.width > 0);
  const cv::Rect border = getBorder(window, cut_window);
  cv::Mat result = input(cut_window);
  if (border != cv::Rect(0, 0, 0, 0)) {
    cv::copyMakeBorder(
      result, result, border.y, border.height, border.x, border.width, border_type);
  }
  return result;
}

inline cv::Mat getGrayImage(cv::Mat image)
{
  cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
  image.convertTo(image, CV_32F, 1.0F / 255.0F);
  return image;
}

}  // namespace RectTools
