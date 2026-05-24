#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include <opencv2/opencv.hpp>
#include <memory>
#include <thread>
#include <vector>

class MotorsNode;
class CameraNode;
class OdometryNode;
class LaserNode;
class BumperNode;

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

    struct LaserData {
        std::vector<float> values;
        double minAngle;
        double maxAngle;
        double minRange;
        double maxRange;
        double timeStamp;
    };

    struct BumperData {
        int state;
        int bumper;
    };

    HAL() = delete;

    static void set_v(const float velocity);
    static void set_w(const float velocity);
    static Pose3d get_pose3d();
    static Pose3d get_odom();
    static cv::Mat get_image();
    static LaserData get_laser_data();
    static BumperData get_bumper_data();

private:
    static void init();
    friend class SystemBootstrapper;

    static std::shared_ptr<MotorsNode> motors_node_;
    static std::shared_ptr<CameraNode> camera_node_;
    static std::shared_ptr<OdometryNode> odometry_node_;
    static std::shared_ptr<OdometryNode> noisy_odometry_node_;
    static std::shared_ptr<LaserNode> laser_node_;
    static std::shared_ptr<BumperNode> bumper_node_;
    static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    static std::thread spin_thread_;
};

#endif