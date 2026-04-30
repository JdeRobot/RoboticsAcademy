#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include <memory>
#include <thread>
#include <vector>
#include <array>

class MotorsNode;
class OdometryNode;
class LaserNode;
namespace rclcpp::executors { class MultiThreadedExecutor; }

class HAL
{
public:
    HAL() = delete;

    static void set_v(const float velocity);
    static void set_w(const float velocity);
    static std::array<double, 3> get_pose3d();
    static std::array<double, 3> get_odom();
    static std::vector<float> get_laser_data();

private:
    static void init();
    friend class SystemBootstrapper;

    static std::shared_ptr<MotorsNode> motors_node_;
    static std::shared_ptr<OdometryNode> odometry_node_;
    static std::shared_ptr<OdometryNode> noisy_odometry_node_;
    static std::shared_ptr<LaserNode> laser_node_;
    static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    static std::thread spin_thread_;
};

#endif