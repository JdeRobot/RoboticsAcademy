#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include <opencv2/opencv.hpp>
#include <memory>
#include <thread>
#include <vector>
#include <string>

// Forward declarations to speed up compilation by avoiding heavy ROS 2 includes.
// - MotorsNode links to "common_interfaces_cpp/hal/motors.hpp"
// - OdometryNode links to "common_interfaces_cpp/hal/odometry.hpp"
// - LaserNode links to "common_interfaces_cpp/hal/laser.hpp"
// - CameraNode links to "common_interfaces_cpp/hal/camera.hpp"
// - NeuralNetwork links to "common_interfaces_cpp/hal/classnet.hpp"

class MotorsNode;
class OdometryNode;
class LaserNode;
class CameraNode;
class NeuralNetwork;
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

    struct BoundingBox {
        int id;
        std::string class_id;
        float score;
        float xmin;
        float ymin;
        float xmax;
        float ymax;
    };

    // Prevent instantiation. HAL acts as a global static utility.
    HAL() = delete;

    static void set_v(const float velocity);
    static void set_w(const float velocity);

    static Pose3d get_pose3d();
    static LaserData get_laser_data();
    static cv::Mat get_image();
    static std::vector<BoundingBox> get_bounding_boxes(const cv::Mat& image);

private:
    static void init();
    friend class SystemBootstrapper;

    // Hidden internal state. Not accessible to the user.
    static std::shared_ptr<MotorsNode> motors_node_;
    static std::shared_ptr<OdometryNode> odometry_node_;
    static std::shared_ptr<LaserNode> laser_node_;
    static std::shared_ptr<CameraNode> camera_node_;
    static std::shared_ptr<NeuralNetwork> neural_network_;
    static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    static std::thread spin_thread_;
};

#endif