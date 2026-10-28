#ifndef HAL_COMMON_HPP_
#define HAL_COMMON_HPP_

// Pieces shared by the UR5 and Dobot HAL, both arms run on the ros2srrc action servers

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "ros2srrc_data/action/move.hpp"
#include "ros2srrc_data/action/robmove.hpp"
#include "ros2srrc_data/msg/robpose.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace pick_place::detail
{

using Move = ros2srrc_data::action::Move;
using Robmove = ros2srrc_data::action::Robmove;
using FJT = control_msgs::action::FollowJointTrajectory;

// Action clients for /Move, /Robmove and the gripper plus the attach topics and the arm state
class ArmNode : public rclcpp::Node
{
public:
  explicit ArmNode(const std::string & graspable_objects);

  // Blocking calls, the background executor fires the result callbacks while they wait
  bool move_execute(const Move::Goal & goal, double wait_time);
  bool robmove_execute(const Robmove::Goal & goal, double wait_time);
  bool gripper_execute(const std::string & joint, double position, double wait_time);

  void publish_auto_attach(bool enabled);

  // The first message can arrive after init so both wait for it
  std::vector<double> joints();
  bool pose(ros2srrc_data::msg::Robpose & out);

private:
  rclcpp_action::Client<Move>::SharedPtr move_client_;
  rclcpp_action::Client<Robmove>::SharedPtr robmove_client_;
  rclcpp_action::Client<FJT>::SharedPtr gripper_client_;
  bool gripper_ready_ = false;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr auto_attach_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr graspable_pub_;
  rclcpp::TimerBase::SharedPtr graspable_timer_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joints_sub_;
  rclcpp::Subscription<ros2srrc_data::msg::Robpose>::SharedPtr pose_sub_;
  std::mutex state_mutex_;
  std::vector<double> joints_;
  ros2srrc_data::msg::Robpose pose_;
  bool has_pose_ = false;
};

// Creates the node and spins it in the background
std::shared_ptr<ArmNode> start(const std::string & graspable_objects);

Robmove::Goal make_robmove_goal(
  const std::string & type, double speed,
  const std::array<double, 3> & xyz, const std::array<double, 3> & ypr);

// Inverse of the quaternion built for Robmove, YPR in degrees in the order MoveLinear takes
std::array<double, 3> quat_to_ypr(double qx, double qy, double qz, double qw);

// Moves of the HAL API, the same for every arm, they print an error when they fail
void move_abs_j(ArmNode & node, const std::vector<double> & joints, double speed, double wait_time);
void move_single_j(
  ArmNode & node, int joint_number, double relative_angle, double speed, double wait_time);
void move_cartesian(
  ArmNode & node, const std::string & type, const std::array<double, 3> & xyz,
  const std::array<double, 3> & ypr, double speed, double wait_time);
void move_rel_linear(
  ArmNode & node, const std::array<double, 3> & xyz, double speed, double wait_time);
void move_rel_reor(
  ArmNode & node, const std::array<double, 3> & ypr, double speed, double wait_time);

std::array<double, 3> tcp_position(ArmNode & node);
std::array<double, 3> tcp_orientation(ArmNode & node);
std::vector<double> joint_states(ArmNode & node);

}  // namespace pick_place::detail

#endif
