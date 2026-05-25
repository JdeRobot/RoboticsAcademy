#include "HAL.hpp"
#include "common_interfaces_cpp/hal/motors.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"
#include "common_interfaces_cpp/hal/laser.hpp"
#include "common_interfaces_cpp/hal/lidar.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

using namespace std::chrono_literals;

std::shared_ptr<MotorsNode> HAL::motors_node_ = nullptr;
std::shared_ptr<OdometryNode> HAL::odometry_node_ = nullptr;
std::shared_ptr<LaserNode> HAL::laser_front_node_ = nullptr;
std::shared_ptr<LaserNode> HAL::laser_right_node_ = nullptr;
std::shared_ptr<LaserNode> HAL::laser_back_node_ = nullptr;
std::shared_ptr<LidarNode> HAL::lidar_node_ = nullptr;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> HAL::executor_ = nullptr;
std::thread HAL::spin_thread_;

void HAL::init()
{
    if (!motors_node_) {
        motors_node_ = std::make_shared<MotorsNode>("/prius_autoparking/cmd_vel", 4.0, 0.3, "hal_motors");
        odometry_node_ = std::make_shared<OdometryNode>("/prius_autoparking/odom", "hal_odom");
        laser_front_node_ = std::make_shared<LaserNode>("/prius_autoparking/scan_front", "hal_laser_front");
        laser_right_node_ = std::make_shared<LaserNode>("/prius_autoparking/scan_side", "hal_laser_right");
        laser_back_node_ = std::make_shared<LaserNode>("/prius_autoparking/scan_back", "hal_laser_back");
        lidar_node_ = std::make_shared<LidarNode>("/prius_autoparking/pc2", "hal_lidar");

        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor_->add_node(motors_node_);
        executor_->add_node(odometry_node_);
        executor_->add_node(laser_front_node_);
        executor_->add_node(laser_right_node_);
        executor_->add_node(laser_back_node_);
        executor_->add_node(lidar_node_);

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

HAL::LaserData HAL::get_front_laser_data()
{
    if (!laser_front_node_) return HAL::LaserData{};
    
    auto raw_laser = laser_front_node_->getLaserData();
    while (raw_laser.values.empty() && rclcpp::ok()) {
        std::this_thread::sleep_for(5ms);
        raw_laser = laser_front_node_->getLaserData();
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

HAL::LaserData HAL::get_right_laser_data()
{
    if (!laser_right_node_) return HAL::LaserData{};
    
    auto raw_laser = laser_right_node_->getLaserData();
    while (raw_laser.values.empty() && rclcpp::ok()) {
        std::this_thread::sleep_for(5ms);
        raw_laser = laser_right_node_->getLaserData();
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

HAL::LaserData HAL::get_back_laser_data()
{
    if (!laser_back_node_) return HAL::LaserData{};
    
    auto raw_laser = laser_back_node_->getLaserData();
    while (raw_laser.values.empty() && rclcpp::ok()) {
        std::this_thread::sleep_for(5ms);
        raw_laser = laser_back_node_->getLaserData();
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

HAL::LidarData HAL::get_lidar_data()
{
    if (!lidar_node_) return HAL::LidarData{};

    auto raw_lidar = lidar_node_->getLidarData();
    while (raw_lidar.points.empty() && rclcpp::ok()) {
        std::this_thread::sleep_for(5ms);
        raw_lidar = lidar_node_->getLidarData();
    }

    return HAL::LidarData{
        raw_lidar.points,
        raw_lidar.intensities,
        raw_lidar.timeStamp,
        raw_lidar.min_range,
        raw_lidar.max_range,
        raw_lidar.field_of_view,
        raw_lidar.is_dense
    };
}