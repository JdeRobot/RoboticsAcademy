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

HAL::Pose3d HAL::get_pose3d()
{
    if (!odometry_node_) return HAL::Pose3d{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    auto raw_pose = odometry_node_->getPose3d();
    return HAL::Pose3d{
        raw_pose.x, 
        raw_pose.y, 
        raw_pose.z, 
        raw_pose.h,
        raw_pose.yaw, 
        raw_pose.pitch, 
        raw_pose.roll,
        raw_pose.timeStamp
    };
}

HAL::Pose3d HAL::get_odom()
{
    if (!noisy_odometry_node_) return HAL::Pose3d{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    auto raw_pose = noisy_odometry_node_->getPose3d();
    return HAL::Pose3d{
        raw_pose.x, 
        raw_pose.y, 
        raw_pose.z, 
        raw_pose.h,
        raw_pose.yaw, 
        raw_pose.pitch, 
        raw_pose.roll,
        raw_pose.timeStamp
    };
}

HAL::BumperData HAL::get_bumper_data()
{
    if (!bumper_node_) return HAL::BumperData{0, 1};
    
    auto raw_data = bumper_node_->getBumperData();
    return HAL::BumperData{
        raw_data.state, 
        raw_data.bumper
    };
}

HAL::LaserData HAL::get_laser_data()
{
    if (!laser_node_) return HAL::LaserData{};
    
    auto raw_laser = laser_node_->getLaserData();
    while (raw_laser.values.empty() && rclcpp::ok()) {
        std::this_thread::sleep_for(5ms);
        raw_laser = laser_node_->getLaserData();
    }
    
    return HAL::LaserData{
        raw_laser.values, 
        raw_laser.minAngle, 
        raw_laser.maxAngle, 
        raw_laser.minRange, 
        raw_laser.maxRange,
        raw_laser.timeStamp
    };
}