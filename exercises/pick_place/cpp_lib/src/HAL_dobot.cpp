#include "HAL_dobot.hpp"
#include "hal_common.hpp"

#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>

using namespace pick_place::detail;

namespace dobot
{

namespace
{

const std::string GRASPABLE_OBJECTS = "red_cube,green_cube,blue_cube";

// The jaws rest closed and open towards negative positions
const std::string GRIPPER_JOINT = "joint_prismatic_l";
constexpr double GRIPPER_OPEN = -0.018;
constexpr double GRIPPER_CLOSED = 0.0;

std::shared_ptr<ArmNode> node;
std::once_flag init_flag;

ArmNode & arm()
{
  std::call_once(
    init_flag, []() {
      std::cout << "HAL (Dobot Magician) initializing" << std::endl;
      node = start(GRASPABLE_OBJECTS);
    });
  return *node;
}

}  // namespace

void HAL::MoveAbsJ(const std::array<double, 4> & joints, double speed, double wait_time)
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

std::array<double, 4> HAL::get_Joint_states()
{
  std::array<double, 4> joints{};
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

  double position = GRIPPER_OPEN + (GRIPPER_CLOSED - GRIPPER_OPEN) * relative_closure / 100.0;

  if (!arm().gripper_execute(GRIPPER_JOINT, position, wait_time)) {
    RCLCPP_ERROR(rclcpp::get_logger("HAL"), "GripperSet failed");
  }
}

void HAL::SuctionSet(bool on, double wait_time)
{
  // The cup has no joint, gz_link_attacher holds whatever touches it while this is on
  arm().publish_auto_attach(on);

  if (wait_time > 0.0) {
    std::this_thread::sleep_for(std::chrono::duration<double>(wait_time));
  }
}

}  // namespace dobot
