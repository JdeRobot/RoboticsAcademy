#include "HAL.hpp"

using namespace std::chrono_literals;

std::shared_ptr<MotorsNode> HAL::motors_node_ = nullptr;
std::shared_ptr<OdometryNode> HAL::odometry_node_ = nullptr;

HAL::HAL() : Node("hal_node")
{
    motors_node_ = std::make_shared<MotorsNode>("/cmd_vel", 4.0, 0.3);
    odometry_node_ = std::make_shared<OdometryNode>("/odom", "hal_odometry_node");

    spin_thread_ = std::thread([]() {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(HAL::motors_node_);
        executor.add_node(HAL::odometry_node_);
        
        while (rclcpp::ok()) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
    spin_thread_.detach();
}

void HAL::set_v(const float speed)
{
    if (motors_node_) {
        motors_node_->sendV(static_cast<double>(speed));
    }
}

void HAL::set_w(const float speed)
{
    if (motors_node_) {
        motors_node_->sendW(static_cast<double>(speed));
    }
}

Pose3d HAL::get_pose3d()
{
    if (odometry_node_) {
        return odometry_node_->getPose3d();
    }
    return Pose3d();
}