#include "HAL.hpp"
#include "common_interfaces_cpp/hal/motors.hpp"
#include "common_interfaces_cpp/hal/camera.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

using namespace std::chrono_literals;

std::shared_ptr<MotorsNode> HAL::motors_node_ = nullptr;
std::shared_ptr<CameraNode> HAL::camera_node_ = nullptr;
std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> HAL::executor_ = nullptr;
std::thread HAL::spin_thread_;

void HAL::init()
{
    if (!motors_node_) {
        motors_node_ = std::make_shared<MotorsNode>("/cmd_vel", 4.0, 0.3, "hal_motors");
        camera_node_ = std::make_shared<CameraNode>("/cam_f1_left/image_raw", "hal_camera");

        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(motors_node_);
        executor_->add_node(camera_node_);

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

cv::Mat HAL::get_image()
{
    if (!camera_node_) return cv::Mat();
    auto image = camera_node_->getImage();
    while (!image && rclcpp::ok()) {
        std::this_thread::sleep_for(5ms);
        image = camera_node_->getImage();
    }
    return (image) ? image->data.clone() : cv::Mat();
}