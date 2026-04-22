#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include <memory>
#include <vector>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "common_interfaces_cpp/hal/motors.hpp"
#include "common_interfaces_cpp/hal/camera.hpp"

class HAL
{
public:
    HAL();
    static void set_v(const float velocity);
    static void set_w(const float velocity);
    static cv::Mat get_image();

    static std::vector<rclcpp::Node::SharedPtr> get_nodes();

private:
    static std::shared_ptr<MotorsNode> motors_node_;
    static std::shared_ptr<CameraNode> camera_node_;
};

#endif