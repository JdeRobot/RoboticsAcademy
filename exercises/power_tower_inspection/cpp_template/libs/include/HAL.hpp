#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include <opencv2/opencv.hpp>
#include <memory>
#include <thread>

// Forward declarations
class DroneWrapper;
class CameraNode;
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

    struct Velocity3d {
        double vx;
        double vy;
        double vz;
        double yaw_rate;
    };

    // Prevent instantiation
    HAL() = delete;

    static cv::Mat get_frontal_image();
    static cv::Mat get_ventral_image();

    static Pose3d get_pose3d();
    static Velocity3d get_velocity();
    static int get_landed_state();

    static void set_cmd_pos(float x, float y, float z, float az);
    static void set_cmd_vel(float vx, float vy, float vz, float az);
    static void set_cmd_mix(float vx, float vy, float z, float az);

    static void takeoff(float h = 3.0f);
    static void land();

private:
    static void init();
    friend class SystemBootstrapper;

    // Hidden internal state
    static std::shared_ptr<DroneWrapper> drone_node_;
    static std::shared_ptr<CameraNode> frontal_camera_node_;
    static std::shared_ptr<CameraNode> ventral_camera_node_;
    static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    static std::thread spin_thread_;
};

#endif