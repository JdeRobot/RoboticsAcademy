#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include <vector>
#include <memory>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "common_interfaces_cpp/hal/motors.hpp"
#include "common_interfaces_cpp/hal/laser.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"

class HAL : public rclcpp::Node
{
public:
  HAL();
  static void setV(const float speed);
  static void setW(const float speed);
  static const LaserData *getLaserData();
  static const Pose3d *getPose3d();
  static const Pose3d *getOdom();

private:
  static std::shared_ptr<MotorsNode> motors_node_;
  static std::shared_ptr<LaserNode> laser_node_;
  static std::shared_ptr<OdometryNode> odometry_node_;
  static std::shared_ptr<OdometryNode> noisy_odometry_node_;
  std::thread spin_thread_;
};

#endif