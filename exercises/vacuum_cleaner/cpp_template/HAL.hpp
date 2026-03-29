#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include <vector>
#include <memory>
#include <chrono>

#include "geometry_msgs/msg/twist.hpp"
#include "gazebo_msgs/msg/contacts_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

// Use specific namespaces to avoid pollution
using namespace std::chrono_literals; // NOLINT
using std::placeholders::_1;

class LaserData
{
public:
  std::vector<float> values;
  float min_angle_ = 0; 
  float max_angle_ = 0;
  float min_range_ = 0;
  float max_range_ = 0;
  float time_stamp_ = 0;

  LaserData(const std::vector<float>& range, float angle_min, float angle_max, float range_max, float range_min)
  {
    values = range;
    // Map ROS angles to JdeRobot coordinate system
        /*
          ROS Angle Map      JdeRobot Angle Map
                0                  PI/2
                |                   |
                |                   |
       PI/2 --------- -PI/2  PI --------- 0
                |                   |
                |                   |
     */
    min_angle_ = angle_min + 3.14 / 2;
    max_angle_ = angle_max + 3.14 / 2;
    min_range_ = range_max;
    max_range_ = range_min;
  }
};

class HAL : public rclcpp::Node
{
public:
  HAL() : Node("hal_node")
  {
    // Initialize Publishers and Subscribers
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/roombaROS/laser/scan", 10,
        std::bind(&HAL::scan_callback, this, _1));
        
    r_bumper_sub_ = this->create_subscription<gazebo_msgs::msg::ContactsState>(
        "/roombaROS/events/right_bumper", 10,
        std::bind(&HAL::right_bumper_callback, this, _1));
        
    c_bumper_sub_ = this->create_subscription<gazebo_msgs::msg::ContactsState>(
        "/roombaROS/events/center_bumper", 10,
        std::bind(&HAL::center_bumper_callback, this, _1));
        
    l_bumper_sub_ = this->create_subscription<gazebo_msgs::msg::ContactsState>(
        "/roombaROS/events/left_bumper", 10,
        std::bind(&HAL::left_bumper_callback, this, _1));
  }

  // Set linear velocity
  static void set_v(const float speed)
  {
    last_twist_.linear.x = speed;
    vel_pub_->publish(last_twist_);
  }

  // Set angular velocity
  static void set_w(const float speed)
  {
    last_twist_.angular.z = speed;
    vel_pub_->publish(last_twist_);
  }

  // Safe memory management using Smart Pointers to prevent leaks
  static std::unique_ptr<LaserData> get_laser_data()
  {
    return std::make_unique<LaserData>(
      last_scan_.ranges, 
      last_scan_.angle_min, 
      last_scan_.angle_max, 
      last_scan_.range_max, 
      last_scan_.range_min
    );
  }

  // Returns current bumper states
  static std::vector<bool> get_bumper_data()
  {
    return {last_right_bumper_, last_center_bumper_, last_left_bumper_};
  }

private:
  // Optimization: Pass by const reference to avoid unnecessary copying
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    last_scan_ = *msg;
  }

  void right_bumper_callback(const gazebo_msgs::msg::ContactsState::SharedPtr msg)
  {
    last_right_bumper_ = !msg->states.empty();
  }

  void center_bumper_callback(const gazebo_msgs::msg::ContactsState::SharedPtr msg)
  {
    last_center_bumper_ = !msg->states.empty();
  }

  void left_bumper_callback(const gazebo_msgs::msg::ContactsState::SharedPtr msg)
  {
    last_left_bumper_ = !msg->states.empty();
  }

  // ROS 2 Communication Objects
  static rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr r_bumper_sub_;
  rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr c_bumper_sub_;
  rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr l_bumper_sub_;

  // Member Variables - Following Google/ROS2 Naming Conventions
  static geometry_msgs::msg::Twist last_twist_;
  static sensor_msgs::msg::LaserScan last_scan_;
  static bool last_right_bumper_;
  static bool last_center_bumper_;
  static bool last_left_bumper_;
};

// Static Members Initialization
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr HAL::vel_pub_ = nullptr;
geometry_msgs::msg::Twist HAL::last_twist_ = geometry_msgs::msg::Twist();
sensor_msgs::msg::LaserScan HAL::last_scan_ = sensor_msgs::msg::LaserScan();
bool HAL::last_right_bumper_ = false;
bool HAL::last_center_bumper_ = false;
bool HAL::last_left_bumper_ = false;

#endif  // INCLUDE_HAL_HPP_