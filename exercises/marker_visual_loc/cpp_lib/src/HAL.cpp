#include "HAL.hpp"

using namespace std::chrono_literals;

std::shared_ptr<MotorsNode> HAL::motor_node_ = nullptr;
std::shared_ptr<CameraNode> HAL::camera_node_ = nullptr;
std::shared_ptr<OdometryNode> HAL::odometry_node_ = nullptr;
std::shared_ptr<LaserNode> HAL::laser_node_ = nullptr;
std::shared_ptr<OdometryNode> HAL::noisy_odometry_node_ = nullptr;

HAL::HAL() : Node("hal_node")
{
    motor_node_ = std::make_shared<MotorsNode>("/turtlebot3/cmd_vel", 4.0, 0.3);
    camera_node_ = std::make_shared<CameraNode>("/turtlebot3/camera/image_raw");
    odometry_node_ = std::make_shared<OdometryNode>("/turtlebot3/odom");
    laser_node_ = std::make_shared<LaserNode>("/turtlebot3/laser/scan");
    noisy_odometry_node_ = std::make_shared<OdometryNode>("/turtlebot3/odom_noisy", "noisy_odometry_node");

    spin_thread_ = std::thread([]() {
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(camera_node_);
        executor.add_node(odometry_node_);
        executor.add_node(laser_node_);
        executor.add_node(noisy_odometry_node_);
        
        while (rclcpp::ok()) {
            executor.spin_some();
            std::this_thread::sleep_for(11ms);
        }
    });
    spin_thread_.detach();
}

Pose3d HAL::getPose3d()
{
    if (odometry_node_) {
        return odometry_node_->getPose3d();
    }
    return Pose3d();
}

Pose3d HAL::getOdom()
{
    if (noisy_odometry_node_) {
        return noisy_odometry_node_->getPose3d();
    }
    return Pose3d();
}

cv::Mat HAL::getImage()
{
    std::shared_ptr<Image> img = nullptr;
    while (img == nullptr && rclcpp::ok()) {
        img = camera_node_->getImage();
        if (img == nullptr) {
            std::this_thread::sleep_for(10ms);
        }
    }
    return img->data;
}

LaserData HAL::getLaserData()
{
    LaserData laser_data = laser_node_->getLaserData();
    while (laser_data.values.empty() && rclcpp::ok()) {
        laser_data = laser_node_->getLaserData();
        if (laser_data.values.empty()) {
            std::this_thread::sleep_for(10ms);
        }
    }
    return laser_data;
}

void HAL::setV(float v)
{
    if (motor_node_) {
        motor_node_->sendV(static_cast<double>(v));
    }
}

void HAL::setW(float w)
{
    if (motor_node_) {
        motor_node_->sendW(static_cast<double>(w));
    }
}