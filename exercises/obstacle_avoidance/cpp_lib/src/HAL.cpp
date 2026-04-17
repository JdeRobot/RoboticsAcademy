#include "HAL.hpp"

using namespace std::chrono_literals;

std::shared_ptr<OdometryNode> HAL::pose3d_node_ = nullptr;
std::shared_ptr<MotorsNode> HAL::motors_node_ = nullptr;
std::shared_ptr<LaserNode> HAL::laser_node_ = nullptr;

HAL::HAL() : Node("hal_node")
{
    pose3d_node_ = std::make_shared<OdometryNode>("/odom");
    motors_node_ = std::make_shared<MotorsNode>("/cmd_vel", 4.0, 0.3);
    laser_node_ = std::make_shared<LaserNode>("/f1/laser/scan");

    spin_thread_ = std::thread([]() {
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(HAL::pose3d_node_);
        executor.add_node(HAL::motors_node_);
        executor.add_node(HAL::laser_node_);
        
        while (rclcpp::ok()) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
    spin_thread_.detach();
}

void HAL::set_v(const double velocity)
{
    if (motors_node_) motors_node_->sendV(velocity);
}

void HAL::set_w(const double velocity)
{
    if (motors_node_) motors_node_->sendW(velocity);
}

Pose3d HAL::get_pose3d()
{
    if (pose3d_node_) return pose3d_node_->getPose3d();
    return Pose3d();
}

const LaserData *HAL::get_laser_data()
{
    if (!laser_node_) return nullptr;
    LaserData data = laser_node_->getLaserData();
    while (data.values.empty() && rclcpp::ok()) {
        data = laser_node_->getLaserData();
    }
    return new LaserData(data);
}