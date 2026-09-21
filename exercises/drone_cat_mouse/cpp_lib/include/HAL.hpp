#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include <opencv2/opencv.hpp>
#include <memory>
#include <thread>
#include <vector>

// Forward declarations
class DroneWrapper;
class CameraNode;
class MousePoseNode;
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

    // The mouse drops out of the sky when it is caught. The cat is flown by
    // student code that knows nothing about that, so the same check runs
    // here and further motion commands stop going through.
    static std::vector<double> get_mouse_position();
    static bool is_caught();

private:
    static void init();
    friend class SystemBootstrapper;

    // Runs the catch/drop check and, the first time it trips, lands the
    // drone. Returns true from that point on, so callers can no-op.
    static bool handle_catch();

    // Hidden internal state
    static std::shared_ptr<DroneWrapper> drone_node_;
    static std::shared_ptr<CameraNode> frontal_camera_node_;
    static std::shared_ptr<CameraNode> ventral_camera_node_;
    static std::shared_ptr<MousePoseNode> mouse_pose_node_;
    static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    static std::thread spin_thread_;
    static bool mouse_flew_;
    static bool stopped_;
};

#endif
