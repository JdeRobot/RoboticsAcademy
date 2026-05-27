#include "HAL.hpp"
#include "common_interfaces_cpp/hal/motors.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"
#include "common_interfaces_cpp/hal/camera.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

using namespace std::chrono_literals;

std::shared_ptr<MotorsNode> HAL::motors_node_ = nullptr;
std::shared_ptr<OdometryNode> HAL::odometry_node_ = nullptr;
std::shared_ptr<CameraNode> HAL::camera_node_ = nullptr;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> HAL::executor_ = nullptr;
std::thread HAL::spin_thread_;

void HAL::init()
{
    if (!motors_node_) {
        motors_node_   = std::make_shared<MotorsNode>("/cmd_vel", 4.0, 0.3, "hal_motors");
        odometry_node_ = std::make_shared<OdometryNode>("/odom", "hal_odom");
        camera_node_   = std::make_shared<CameraNode>("/prius_autoparking/image_raw", "hal_camera");

        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor_->add_node(motors_node_);
        executor_->add_node(odometry_node_);
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
