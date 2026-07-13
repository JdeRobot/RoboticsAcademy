#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include <array>
#include <memory>
#include <thread>

// Forward declarations to keep ROS 2 headers out of user-facing code.
class HALNode;
namespace rclcpp::executors { class MultiThreadedExecutor; }

class HAL
{
public:
    // Prevent instantiation. HAL acts as a global static utility.
    HAL() = delete;

    // Absolute joint movement. joints[6] in degrees, speed in [0,1], wait_time in seconds.
    static void MoveAbsJ(const std::array<double, 6>& joints, double speed, double wait_time);

    // Relative single-joint movement. joint_number in [1,6], relative_angle in degrees.
    static void MoveSingleJ(int joint_number, double relative_angle, double speed, double wait_time);

    // Absolute linear (Cartesian) move. xyz in metres, ypr in degrees.
    static void MoveLinear(const std::array<double, 3>& xyz, const std::array<double, 3>& ypr, double speed, double wait_time);

    // Absolute point-to-point (Cartesian) move. xyz in metres, ypr in degrees.
    static void MoveJoint(const std::array<double, 3>& xyz, const std::array<double, 3>& ypr, double speed, double wait_time);

    // Relative linear Cartesian increment. xyz in metres.
    static void MoveRelLinear(const std::array<double, 3>& xyz, double speed, double wait_time);

    // Relative TCP reorientation. ypr in degrees.
    static void MoveRelReor(const std::array<double, 3>& ypr, double speed, double wait_time);

    // Suction gripper control. on = true grips a graspable object in contact
    // with the cup, on = false releases the held object. wait_time in seconds.
    static void SuctionSet(bool on, double wait_time);

private:
    static void init();
    friend class SystemBootstrapper;

    // Hidden internal state. Not accessible to the user.
    static std::shared_ptr<HALNode> hal_node_;
    static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    static std::thread spin_thread_;
};

#endif
