#include "HAL.hpp"
#include "hal_common.hpp"

#include <iostream>
#include <mutex>

using namespace pick_place::detail;

namespace ur5
{

namespace
{

const std::string GRASPABLE_OBJECTS = "blue_ball,green_cylinder,yellow_box,red_box";
const std::string GRIPPER_JOINT = "robotiq_85_left_knuckle_joint";

std::shared_ptr<ArmNode> node;
std::once_flag init_flag;

ArmNode & arm()
{
  std::call_once(
    init_flag, []() {
      std::cout << "HAL (UR5) initializing" << std::endl;
      node = start(GRASPABLE_OBJECTS);
    });
  return *node;
}

}  // namespace

void HAL::MoveAbsJ(const std::array<double, 6> & joints, double speed, double wait_time)
{
  move_abs_j(arm(), {joints.begin(), joints.end()}, speed, wait_time);
}

void HAL::MoveSingleJ(int joint_number, double relative_angle, double speed, double wait_time)
{
  move_single_j(arm(), joint_number, relative_angle, speed, wait_time);
}

void HAL::MoveLinear(
  const std::array<double, 3> & xyz, const std::array<double, 3> & ypr,
  double speed, double wait_time)
{
  move_cartesian(arm(), "LIN", xyz, ypr, speed, wait_time);
}

void HAL::MoveJoint(
  const std::array<double, 3> & xyz, const std::array<double, 3> & ypr,
  double speed, double wait_time)
{
  move_cartesian(arm(), "PTP", xyz, ypr, speed, wait_time);
}

void HAL::MoveRelLinear(const std::array<double, 3> & xyz, double speed, double wait_time)
{
  move_rel_linear(arm(), xyz, speed, wait_time);
}

void HAL::MoveRelReor(const std::array<double, 3> & ypr, double speed, double wait_time)
{
  move_rel_reor(arm(), ypr, speed, wait_time);
}

std::array<double, 3> HAL::get_TCP_position()
{
  return tcp_position(arm());
}

std::array<double, 3> HAL::get_TCP_orientation()
{
  return tcp_orientation(arm());
}

std::array<double, 6> HAL::get_Joint_states()
{
  std::array<double, 6> joints{};
  auto values = joint_states(arm());
  for (size_t i = 0; i < values.size() && i < joints.size(); i++) {
    joints[i] = values[i];
  }
  return joints;
}

void HAL::GripperSet(double relative_closure, double wait_time)
{
  // Enable contact-based auto-attach when closing, release when opening.
  arm().publish_auto_attach(relative_closure > 5.0);

  // Convert percentage [0,100] to joint position [0.0,1.0]
  double position = relative_closure / 100.0;

  if (!arm().gripper_execute(GRIPPER_JOINT, position, wait_time)) {
    RCLCPP_ERROR(rclcpp::get_logger("HAL"), "GripperSet failed");
  }
}

}  // namespace ur5
