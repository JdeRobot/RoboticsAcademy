#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include <memory>
#include <thread>
#include <vector>
#include <array>
#include <string>

// Forward declarations to avoid heavy includes and internal data types.
class MotorsNode;
class OdometryNode;
class LaserNode;
class BumperNode;
namespace rclcpp::executors { class MultiThreadedExecutor; }

class HAL
{
public:
    HAL() = delete;

    static void set_v(const float velocity);
    static void set_w(const float velocity);

    // Returns the current robot pose: [0] = x, [1] = y, [2] = yaw
    static std::array<double, 3> get_pose3d();

    // Returns the noisy odometry pose: [0] = x, [1] = y, [2] = yaw
    static std::array<double, 3> get_odom();

    // Returns bumper state: [0] = state (1 collision, 0 free), [1] = bumper ID (0 right, 1 center, 2 left)
    static std::array<int, 2> get_bumper_data();

    // Returns the laser range distances
    static std::vector<float> get_laser_data();

private:
    static void init();
    friend class SystemBootstrapper;

    static std::shared_ptr<MotorsNode> motors_node_;
    static std::shared_ptr<OdometryNode> odometry_node_;
    static std::shared_ptr<OdometryNode> noisy_odometry_node_;
    static std::shared_ptr<LaserNode> laser_node_;
    static std::shared_ptr<BumperNode> bumper_node_;
    static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    static std::thread spin_thread_;
};

#endif