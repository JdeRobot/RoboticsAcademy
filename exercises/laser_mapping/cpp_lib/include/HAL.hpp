#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include <memory>
#include <thread>
#include <vector>

// Forward declarations to speed up compilation by avoiding heavy ROS 2 includes.
// - MotorsNode links to "common_interfaces_cpp/hal/motors.hpp"
// - OdometryNode links to "common_interfaces_cpp/hal/odometry.hpp"
// - LaserNode links to "common_interfaces_cpp/hal/laser.hpp"

class MotorsNode;
class OdometryNode;
class LaserNode;
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

    // Prevent instantiation. HAL acts as a global static utility.
    HAL() = delete;

    static void set_v(const float velocity);
    static void set_w(const float velocity);
    static Pose3d get_pose3d();
    static Pose3d get_odom();
    static LaserData get_laser_data();

private:
    static void init();
    friend class SystemBootstrapper;

    // Hidden internal state. Not accessible to the user.
    static std::shared_ptr<MotorsNode> motors_node_;
    static std::shared_ptr<OdometryNode> odometry_node_;
    static std::shared_ptr<OdometryNode> noisy_odometry_node_;
    static std::shared_ptr<LaserNode> laser_node_;
    static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    static std::thread spin_thread_;
};

#endif