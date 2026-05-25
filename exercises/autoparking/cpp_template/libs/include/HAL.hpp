#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include <memory>
#include <thread>
#include <vector>
#include <array>
#include <utility>

// Forward declarations to speed up compilation by avoiding heavy ROS 2 includes.
// - MotorsNode links to "common_interfaces_cpp/hal/motors.hpp"
// - OdometryNode links to "common_interfaces_cpp/hal/odometry.hpp"
// - LaserNode links to "common_interfaces_cpp/hal/laser.hpp"
// - LidarNode links to "common_interfaces_cpp/hal/lidar.hpp"
class MotorsNode;
class OdometryNode;
class LaserNode;
class LidarNode;

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

    struct LidarData {
        std::vector<std::array<float, 3>> points;
        std::vector<float> intensities;
        double timeStamp;
        double min_range;
        double max_range;
        std::pair<double, double> field_of_view;
        bool is_dense;
    };

    // Prevent instantiation. HAL acts as a global static utility.
    HAL() = delete;

    static void set_v(const float velocity);
    static void set_w(const float velocity);
    static Pose3d get_pose3d();
    static LaserData get_front_laser_data();
    static LaserData get_right_laser_data();
    static LaserData get_back_laser_data();
    static LidarData get_lidar_data();

private:
    static void init();
    friend class SystemBootstrapper;

    // Hidden internal state. Not accessible to the user.
    static std::shared_ptr<MotorsNode> motors_node_;
    static std::shared_ptr<OdometryNode> odometry_node_;
    static std::shared_ptr<LaserNode> laser_front_node_;
    static std::shared_ptr<LaserNode> laser_right_node_;
    static std::shared_ptr<LaserNode> laser_back_node_;
    static std::shared_ptr<LidarNode> lidar_node_;
    static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    static std::thread spin_thread_;
};

#endif