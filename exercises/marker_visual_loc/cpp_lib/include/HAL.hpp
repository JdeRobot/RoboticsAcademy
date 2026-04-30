#ifndef HAL_HPP_
#define HAL_HPP_

#include <memory>
#include <thread>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "common_interfaces_cpp/hal/motors.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"
#include "common_interfaces_cpp/hal/laser.hpp"
#include "common_interfaces_cpp/hal/camera.hpp"

class HAL : public rclcpp::Node {
public:
    HAL();

    static Pose3d getPose3d();
    static Pose3d getOdom();
    static cv::Mat getImage();
    static LaserData getLaserData();

    static void setV(float v);
    static void setW(float w);

private:
    static std::shared_ptr<MotorsNode> motor_node_;
    static std::shared_ptr<CameraNode> camera_node_;
    static std::shared_ptr<OdometryNode> odometry_node_;
    static std::shared_ptr<LaserNode> laser_node_;
    static std::shared_ptr<OdometryNode> noisy_odometry_node_;

    std::thread spin_thread_;
};

#endif