#ifndef INCLUDE_HAL_DOBOT_HPP_
#define INCLUDE_HAL_DOBOT_HPP_

#include <array>

// HAL for the Dobot Magician with the gripper or the suction cup
// It starts the first time one of its functions is called
namespace dobot
{

class HAL
{
public:
    // Prevent instantiation. HAL acts as a global static utility.
    HAL() = delete;

    // Absolute joint movement. joints[4] in degrees, speed in [0,1], wait_time in seconds.
    static void MoveAbsJ(const std::array<double, 4>& joints, double speed, double wait_time);

    // Relative single-joint movement. joint_number in [1,4], relative_angle in degrees.
    static void MoveSingleJ(int joint_number, double relative_angle, double speed, double wait_time);

    // Absolute linear (Cartesian) move. xyz in metres, ypr in degrees.
    static void MoveLinear(const std::array<double, 3>& xyz, const std::array<double, 3>& ypr, double speed, double wait_time);

    // Absolute point-to-point (Cartesian) move. xyz in metres, ypr in degrees.
    static void MoveJoint(const std::array<double, 3>& xyz, const std::array<double, 3>& ypr, double speed, double wait_time);

    // Relative linear Cartesian increment. xyz in metres.
    static void MoveRelLinear(const std::array<double, 3>& xyz, double speed, double wait_time);

    // Relative TCP reorientation. ypr in degrees.
    static void MoveRelReor(const std::array<double, 3>& ypr, double speed, double wait_time);

    // TCP pose in the same frame and angle order MoveLinear takes. Position in metres, orientation in degrees.
    static std::array<double, 3> get_TCP_position();
    static std::array<double, 3> get_TCP_orientation();

    // Current joint angles in degrees, in the order MoveAbsJ takes them.
    static std::array<double, 4> get_Joint_states();

    // Gripper control. relative_closure in [0,100] (0 = fully open, 100 = fully closed).
    static void GripperSet(double relative_closure, double wait_time);

    // Suction cup control. on grips any object touching the cup, off releases it.
    static void SuctionSet(bool on, double wait_time);
};

}  // namespace dobot

using HAL = dobot::HAL;

#endif
