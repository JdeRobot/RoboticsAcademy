#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include <opencv2/opencv.hpp>
#include <memory>
#include <thread>
#include <vector>

// Forward declarations to speed up compilation by avoiding heavy ROS 2 includes.
// - MotorsNode   links to "common_interfaces_cpp/hal/motors.hpp"
// - CameraNode   links to "common_interfaces_cpp/hal/camera.hpp"
// - OdometryNode links to "common_interfaces_cpp/hal/odometry.hpp"
class MotorsNode;
class CameraNode;
class OdometryNode;
class ArmedNode;
namespace rclcpp::executors { class MultiThreadedExecutor; }

class HAL
{
public:
    struct Pose3d {
        double x;
        double y;
        double z;
        double h;
        double yaw;
        double pitch;
        double roll;
        double timeStamp;
    };

    HAL() = delete;

    static void set_v(const float velocity);
    static void set_w(const float velocity);
    static cv::Mat get_image();

    static Pose3d get_pose3d();

    static Pose3d get_rival_pose3d();
    static std::vector<double> get_rival_position();

    // Distance to the rival in the ground plane
    static double get_gap();

    static bool is_caught();

    // False until the rival has pulled clear of the grid
    static bool is_racing();

private:
    static void init();
    friend class SystemBootstrapper;

    // Stops the car the first time the catch trips
    static bool handle_catch();

    // Hidden internal state. Not accessible to the user.
    static std::shared_ptr<MotorsNode> motors_node_;
    static std::shared_ptr<CameraNode> camera_node_;
    static std::shared_ptr<OdometryNode> odom_node_;
    static std::shared_ptr<OdometryNode> rival_odom_node_;
    static std::shared_ptr<ArmedNode> armed_node_;
    static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    static std::thread spin_thread_;
    static bool stopped_;
};

#endif
