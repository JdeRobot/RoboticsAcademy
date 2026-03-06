#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "gazebo_msgs/msg/contacts_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"


using namespace std::chrono_literals; // NOLINT
using namespace std;                  // NOLINT
using std::placeholders::_1;

class LaserData
{
public:
  vector<float> values;
  float minAngle = 0;
  float maxAngle = 0;
  float minRange = 0;
  float maxRange = 0;
  float timeStamp = 0;

  LaserData(vector<float> range, float angle_min, float angle_max, float range_max, float range_min)
  {
    values = range;
    /*
          ROS Angle Map      JdeRobot Angle Map
                0                  PI/2
                |                   |
                |                   |
       PI/2 --------- -PI/2  PI --------- 0
                |                   |
                |                   |
     */
    minAngle = angle_min + 3.14 / 2;
    maxAngle = angle_max + 3.14 / 2;
    minRange = range_max;
    maxRange = range_min;
  }

private:
};

class HAL : public rclcpp::Node
{
public:
  HAL() : Node("hal_node")
  {
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/roombaROS/laser/scan", 10,
        std::bind(&HAL::scan_callback, this, _1));
    r_bumper_sub_ = create_subscription<gazebo_msgs::msg::ContactsState>(
        "/roombaROS/events/right_bumper", 10,
        std::bind(&HAL::right_bumper_callback, this, _1));
    c_bumper_sub_ = create_subscription<gazebo_msgs::msg::ContactsState>(
        "/roombaROS/events/center_bumper", 10,
        std::bind(&HAL::center_bumper_callback, this, _1));
    l_bumper_sub_ = create_subscription<gazebo_msgs::msg::ContactsState>(
        "/roombaROS/events/left_bumper", 10,
        std::bind(&HAL::left_bumper_callback, this, _1));
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

  static const LaserData *get_laser_data()
  {
    // int n = sizeof(last_scan_->ranges) / sizeof(last_scan_->ranges[0]);
    // vector<float> range(last_scan_->ranges, last_scan_->ranges + n);
    const LaserData *laser = new LaserData(last_scan_.ranges, last_scan_.angle_min, last_scan_.angle_max, last_scan_.range_max, last_scan_.range_min);
    return laser;
  };

  static vector<bool> get_bumper_data()
  {
    vector<bool> v = {last_rigth_bumper, last_center_bumper, last_left_bumper};
    return v;
  };

private:
  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
  {
    last_scan_ = *msg;
  }

  void right_bumper_callback(gazebo_msgs::msg::ContactsState::UniquePtr msg)
  {
    last_rigth_bumper = size(msg->states) > 0;
  };

  void center_bumper_callback(gazebo_msgs::msg::ContactsState::UniquePtr msg)
  {
    last_center_bumper = size(msg->states) > 0;
  };

  void left_bumper_callback(gazebo_msgs::msg::ContactsState::UniquePtr msg)
  {
    last_left_bumper = size(msg->states) > 0;
  };

  // Publisher
  static rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr r_bumper_sub_;
  rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr c_bumper_sub_;
  rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr l_bumper_sub_;

  // Message
  static geometry_msgs::msg::Twist last_twist;
  static sensor_msgs::msg::LaserScan last_scan_;
  static bool last_rigth_bumper;
  static bool last_center_bumper;
  static bool last_left_bumper;
};

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr HAL::vel_pub_ = nullptr;
geometry_msgs::msg::Twist HAL::last_twist = geometry_msgs::msg::Twist();
sensor_msgs::msg::LaserScan HAL::last_scan_ = sensor_msgs::msg::LaserScan();
bool HAL::last_rigth_bumper = false;
bool HAL::last_center_bumper = false;
bool HAL::last_left_bumper = false;

#endif