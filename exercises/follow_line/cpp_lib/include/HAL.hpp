#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include <memory>
#include <thread>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "common_interfaces_cpp/hal/motors.hpp"
#include "common_interfaces_cpp/hal/camera.hpp"

class HAL : public rclcpp::Node
{
public:
    HAL();
    static void setV(const float velocity);
    static void setW(const float velocity);
    static std::shared_ptr<Image> getImage();

private:
    static std::shared_ptr<MotorsNode> motors_node_;
    static std::shared_ptr<CameraNode> camera_node_;
    std::thread spin_thread_;
};

#endif