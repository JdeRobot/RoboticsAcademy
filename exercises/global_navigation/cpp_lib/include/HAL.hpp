#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include <memory>
#include <thread>

class MotorsNode;
class OdometryNode;
namespace rclcpp::executors { class SingleThreadedExecutor; }

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
    static Pose3d get_pose3d();

private:
    static void init();
    friend class SystemBootstrapper;

    static std::shared_ptr<MotorsNode> motors_node_;
    static std::shared_ptr<OdometryNode> odometry_node_;
    static std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    static std::thread spin_thread_;
};

#endif