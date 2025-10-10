#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "gazebo_msgs/msg/performance_metrics.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/odometry.hpp"
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
        "/cam_f1_left/image_raw", 10,
        std::bind(&HAL::topic_callback_info, this, _1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&HAL::pose_callback, this, _1));
    perf_sub_ = create_subscription<gazebo_msgs::msg::PerformanceMetrics>(
        "/performance_metrics", 10,
        std::bind(&HAL::performance_callback, this, _1));
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

  static vector<double> get_pose()
  {
    vector<double> v = {last_odom.pose.pose.position.x, last_odom.pose.pose.position.y, last_odom.pose.pose.position.z};
    return v;
  };

  static double get_performance()
  {
    return last_perf.real_time_factor;
  };

private:
  void topic_callback_info(sensor_msgs::msg::Image::UniquePtr msg)
  {
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

  void pose_callback(nav_msgs::msg::Odometry::UniquePtr msg)
  {
    last_odom = *msg;
  };

  void performance_callback(gazebo_msgs::msg::PerformanceMetrics::UniquePtr msg)
  {
    last_perf = *msg;
  };

  // Publisher
  static rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<gazebo_msgs::msg::PerformanceMetrics>::SharedPtr perf_sub_;

  // Message
  static geometry_msgs::msg::Twist last_twist;
  static cv::Mat image_rgb;
  static nav_msgs::msg::Odometry last_odom;
  static gazebo_msgs::msg::PerformanceMetrics last_perf;
};

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr HAL::vel_pub_ = nullptr;
geometry_msgs::msg::Twist HAL::last_twist = geometry_msgs::msg::Twist();
nav_msgs::msg::Odometry HAL::last_odom = nav_msgs::msg::Odometry();
gazebo_msgs::msg::PerformanceMetrics HAL::last_perf = gazebo_msgs::msg::PerformanceMetrics();
cv::Mat HAL::image_rgb = cv::Mat();

#endif