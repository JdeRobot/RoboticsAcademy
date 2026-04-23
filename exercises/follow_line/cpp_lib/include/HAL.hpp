#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include <opencv2/opencv.hpp>
#include <memory>
#include <thread>

// Forward declarations to speed up compilation by avoiding heavy ROS 2 includes.
// - MotorsNode links to "common_interfaces_cpp/hal/motors.hpp"
// - CameraNode links to "common_interfaces_cpp/hal/camera.hpp"
class MotorsNode;
class CameraNode;
namespace rclcpp::executors { class SingleThreadedExecutor; }

class HAL
{
public:
    // Prevent instantiation. HAL acts as a global static utility.
    HAL() = delete;

    static void set_v(const float velocity);
    static void set_w(const float velocity);
    static cv::Mat get_image();

private:
    static void init();
    friend class SystemBootstrapper;

    // Hidden internal state. Not accessible to the user.
    static std::shared_ptr<MotorsNode> motors_node_;
    static std::shared_ptr<CameraNode> camera_node_;
    static std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    static std::thread spin_thread_;
};

#endif