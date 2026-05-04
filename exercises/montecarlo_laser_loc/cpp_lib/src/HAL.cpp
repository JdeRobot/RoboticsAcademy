#include "HAL.hpp"
#include "common_interfaces_cpp/hal/motors.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"
#include "common_interfaces_cpp/hal/laser.hpp"
#include "common_interfaces_cpp/hal/bumper.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

using namespace std::chrono_literals;

std::shared_ptr<MotorsNode> HAL::motors_node_ = nullptr;
std::shared_ptr<OdometryNode> HAL::odometry_node_ = nullptr;
std::shared_ptr<OdometryNode> HAL::noisy_odometry_node_ = nullptr;
std::shared_ptr<LaserNode> HAL::laser_node_ = nullptr;
std::shared_ptr<BumperNode> HAL::bumper_node_ = nullptr;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> HAL::executor_ = nullptr;
std::thread HAL::spin_thread_;

void HAL::init()
{
    if (!motors_node_) {
        motors_node_ = std::make_shared<MotorsNode>("/cmd_vel", 4.0, 0.3, "hal_motors");
        odometry_node_ = std::make_shared<OdometryNode>("/odom", "hal_odometry");
        noisy_odometry_node_ = std::make_shared<OdometryNode>("/odom_noisy", "noisy_odometry_node");
        laser_node_ = std::make_shared<LaserNode>("/roombaROS/laser/scan", "hal_laser");
        
        std::vector<std::string> bumper_topics = {
            "/roombaROS/events/right_bumper",
            "/roombaROS/events/center_bumper",
            "/roombaROS/events/left_bumper"
        };
        bumper_node_ = std::make_shared<BumperNode>(bumper_topics, "hal_bumper");

        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor_->add_node(motors_node_);
        executor_->add_node(odometry_node_);
        executor_->add_node(noisy_odometry_node_);
        executor_->add_node(laser_node_);
        executor_->add_node(bumper_node_);

        spin_thread_ = std::thread([]() {
            executor_->spin();
        });
        spin_thread_.detach();
    }
}

void HAL::set_v(const float velocity)
{
    if (motors_node_) motors_node_->sendV(static_cast<double>(velocity));
}

void HAL::set_w(const float velocity)
{
    if (motors_node_) motors_node_->sendW(static_cast<double>(velocity));
}

std::array<double, 3> HAL::get_pose3d()
{
    if (!odometry_node_) return {0.0, 0.0, 0.0};
    auto pose = odometry_node_->getPose3d();
    return {pose.x, pose.y, pose.yaw};
}

std::array<double, 3> HAL::get_odom()
{
    if (!noisy_odometry_node_) return {0.0, 0.0, 0.0};
    auto pose = noisy_odometry_node_->getPose3d();
    return {pose.x, pose.y, pose.yaw};
}

std::array<int, 2> HAL::get_bumper_data()
{
    if (!bumper_node_) return {0, 1}; // Default safe state (no collision, center)
    auto data = bumper_node_->getBumperData();
    return {data.state, data.bumper};
}

std::vector<float> HAL::get_laser_data()
{
    if (!laser_node_) return std::vector<float>();
    auto laser_data = laser_node_->getLaserData();
    while (laser_data.values.empty() && rclcpp::ok()) {
        std::this_thread::sleep_for(5ms);
        laser_data = laser_node_->getLaserData();
    }
    return laser_data.values;
}