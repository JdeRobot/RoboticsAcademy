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
    static void set_v(const double velocity);
    static void set_w(const double velocity);
    static Pose3d get_pose3d();
    static const LaserData *get_laser_data();

private:
    static std::shared_ptr<OdometryNode> pose3d_node_;
    static std::shared_ptr<MotorsNode> motors_node_;
    static std::shared_ptr<LaserNode> laser_node_;
    std::thread spin_thread_;
};

#endif