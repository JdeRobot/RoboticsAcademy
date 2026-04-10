#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include <vector>
#include <memory>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "common_interfaces_cpp/hal/motors.hpp"
#include "common_interfaces_cpp/hal/laser.hpp"
#include "common_interfaces_cpp/hal/bumper.hpp"

class HAL : public rclcpp::Node
{
public:
  HAL();
  static void set_v(const float speed);
  static void set_w(const float speed);
  static const LaserData *get_laser_data();
  static std::vector<bool> get_bumper_data();

private:
  static std::shared_ptr<MotorsNode> motors_node_;
  static std::shared_ptr<LaserNode> laser_node_;
  static std::shared_ptr<BumperNode> bumper_node_;
  std::thread spin_thread_;
};

#endif