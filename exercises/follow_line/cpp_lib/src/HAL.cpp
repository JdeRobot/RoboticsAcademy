#include "HAL.hpp"
#include <chrono>

using namespace std::chrono_literals;

std::shared_ptr<MotorsNode> HAL::motors_node_ = nullptr;
std::shared_ptr<CameraNode> HAL::camera_node_ = nullptr;

HAL::HAL() : Node("hal_node")
{
    motors_node_ = std::make_shared<MotorsNode>("/cmd_vel", 4.0, 0.3);
    camera_node_ = std::make_shared<CameraNode>("/cam_f1_left/image_raw");

    spin_thread_ = std::thread([]() {
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(HAL::motors_node_);
        executor.add_node(HAL::camera_node_);
        
        while (rclcpp::ok()) {
            executor.spin_some();
            std::this_thread::sleep_for(11ms);
        }
    });
    spin_thread_.detach();
}

void HAL::setV(const float velocity)
{
    if (motors_node_) {
        motors_node_->sendV(static_cast<double>(velocity));
    }
}

void HAL::setW(const float velocity)
{
    if (motors_node_) {
        motors_node_->sendW(static_cast<double>(velocity));
    }
}

std::shared_ptr<Image> HAL::getImage()
{
    if (!camera_node_) {
        return nullptr;
    }
    auto image = camera_node_->getImage();
    while (!image && rclcpp::ok()) {
        std::this_thread::sleep_for(5ms);
        image = camera_node_->getImage();
    }
    return image;
}