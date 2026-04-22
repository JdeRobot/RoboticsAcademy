#include "HAL.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

std::shared_ptr<MotorsNode> HAL::motors_node_ = nullptr;
std::shared_ptr<CameraNode> HAL::camera_node_ = nullptr;

HAL::HAL()
{
    motors_node_ = std::make_shared<MotorsNode>("/cmd_vel", 4.0, 0.3);
    camera_node_ = std::make_shared<CameraNode>("/cam_f1_left/image_raw");
}

void HAL::set_v(const float velocity)
{
    if (motors_node_) {
        motors_node_->sendV(static_cast<double>(velocity));
    }
}

void HAL::set_w(const float velocity)
{
    if (motors_node_) {
        motors_node_->sendW(static_cast<double>(velocity));
    }
}

cv::Mat HAL::get_image()
{
    if (!camera_node_) {
        return cv::Mat();
    }

    auto image = camera_node_->getImage();

    while (!image && rclcpp::ok()) {
        std::this_thread::sleep_for(5ms);
        image = camera_node_->getImage();
    }

    if (!image) {
        return cv::Mat();
    }

    return image->data.clone();
}

std::vector<rclcpp::Node::SharedPtr> HAL::get_nodes()
{
    std::vector<rclcpp::Node::SharedPtr> nodes;
    if (motors_node_) {
        nodes.push_back(motors_node_);
    }
    if (camera_node_) {
        nodes.push_back(camera_node_);
    }
    return nodes;
}