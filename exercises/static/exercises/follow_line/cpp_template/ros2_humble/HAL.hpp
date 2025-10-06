#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

using namespace std::chrono_literals; // NOLINT
using namespace std;                  // NOLINT
using std::placeholders::_1;

class HAL : public rclcpp::Node
{
public:
  HAL() : Node("hal_node")
  {
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    cam_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/cam_f1_left/image_raw", 1,
        std::bind(&HAL::topic_callback_info, this, _1));
  };

  static void set_v(const float speed)
  {
    last_twist.linear.x = speed;
    vel_pub_->publish(last_twist);
  };

  static void set_w(const float speed)
  {
    last_twist.angular.z = speed;
    vel_pub_->publish(last_twist);
  };

  static cv::Mat get_image()
  {
    return image_rgb;
  };

private:
  void topic_callback_info(sensor_msgs::msg::Image::UniquePtr msg)
  {
    cout << "Image received" << endl;
    // Convert ROS Image to OpenCV Image | sensor_msgs::msg::Image -> cv::Mat
    cv_bridge::CvImagePtr image_rgb_ptr;
    try
    {
      image_rgb_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat image_rgb_raw = image_rgb_ptr->image;
    image_rgb = image_rgb_raw;
  };

  // Publisher
  static rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_sub_;

  // Message
  static geometry_msgs::msg::Twist last_twist;
  static cv::Mat image_rgb;
};

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr HAL::vel_pub_ = nullptr;
geometry_msgs::msg::Twist HAL::last_twist = geometry_msgs::msg::Twist();
cv::Mat HAL::image_rgb = cv::Mat();

#endif