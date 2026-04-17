#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include <memory>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "common_interfaces_cpp/hal/motors.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"

class HAL : public rclcpp::Node
{
public:
    HAL();
    static void set_v(const float speed);
    static void set_w(const float speed);
    static Pose3d get_pose3d();

private:
    static std::shared_ptr<MotorsNode> motors_node_;
    static std::shared_ptr<OdometryNode> odometry_node_;
    std::thread spin_thread_;
};

#endif