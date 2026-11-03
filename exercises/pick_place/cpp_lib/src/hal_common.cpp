#include "hal_common.hpp"

#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <thread>

using namespace std::chrono_literals;

namespace pick_place::detail
{

ArmNode::ArmNode(const std::string & graspable_objects)
: rclcpp::Node("hal_node")
{
  move_client_ = rclcpp_action::create_client<Move>(this, "/Move");
  robmove_client_ = rclcpp_action::create_client<Robmove>(this, "/Robmove");
  gripper_client_ = rclcpp_action::create_client<FJT>(
    this, "/gripper_controller/follow_joint_trajectory");

  auto_attach_pub_ = create_publisher<std_msgs::msg::Bool>("/gripper_auto_attach", 10);
  graspable_pub_ = create_publisher<std_msgs::msg::String>("/graspable_objects", 10);

  joints_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
    "/Robjoints", 1,
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      joints_ = msg->data;
    });
  pose_sub_ = create_subscription<ros2srrc_data::msg::Robpose>(
    "/Robpose", 1,
    [this](const ros2srrc_data::msg::Robpose::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      pose_ = *msg;
      has_pose_ = true;
    });

  RCLCPP_INFO(get_logger(), "Waiting for /Move action server...");
  while (!move_client_->wait_for_action_server(1s)) {
    RCLCPP_INFO(get_logger(), "Waiting for /Move...");
  }

  RCLCPP_INFO(get_logger(), "Waiting for /Robmove action server...");
  while (!robmove_client_->wait_for_action_server(1s)) {
    RCLCPP_INFO(get_logger(), "Waiting for /Robmove...");
  }

  std_msgs::msg::String msg;
  msg.data = graspable_objects;
  graspable_pub_->publish(msg);

  // Periodic republish so late-joining subscribers receive the list
  graspable_timer_ = create_wall_timer(
    1s, [this, msg]() {graspable_pub_->publish(msg);});

  RCLCPP_INFO(get_logger(), "HAL ready");
}

bool ArmNode::move_execute(const Move::Goal & goal, double wait_time)
{
  std::promise<bool> prom;
  auto fut = prom.get_future();

  auto opts = rclcpp_action::Client<Move>::SendGoalOptions();
  opts.result_callback =
    [&prom](const rclcpp_action::ClientGoalHandle<Move>::WrappedResult & r) {
      prom.set_value(r.result->result.find("FAILED") == std::string::npos);
    };

  move_client_->async_send_goal(goal, opts);
  bool ok = fut.get();

  if (wait_time > 0.0) {
    std::this_thread::sleep_for(std::chrono::duration<double>(wait_time));
  }
  return ok;
}

bool ArmNode::robmove_execute(const Robmove::Goal & goal, double wait_time)
{
  std::promise<bool> prom;
  auto fut = prom.get_future();

  auto opts = rclcpp_action::Client<Robmove>::SendGoalOptions();
  opts.result_callback =
    [&prom](const rclcpp_action::ClientGoalHandle<Robmove>::WrappedResult & r) {
      prom.set_value(r.result->success);
    };

  robmove_client_->async_send_goal(goal, opts);
  bool ok = fut.get();

  if (wait_time > 0.0) {
    std::this_thread::sleep_for(std::chrono::duration<double>(wait_time));
  }
  return ok;
}

bool ArmNode::gripper_execute(const std::string & joint, double position, double wait_time)
{
  // Waits on first use so arms without a gripper controller never block on it
  if (!gripper_ready_) {
    RCLCPP_INFO(get_logger(), "Waiting for gripper action server...");
    while (!gripper_client_->wait_for_action_server(1s)) {
      RCLCPP_INFO(get_logger(), "Waiting for gripper controller...");
    }
    gripper_ready_ = true;
  }

  FJT::Goal goal;
  goal.trajectory.joint_names = {joint};

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {position};
  point.time_from_start.sec = 1;
  goal.trajectory.points.push_back(point);

  std::promise<bool> prom;
  auto fut = prom.get_future();

  auto opts = rclcpp_action::Client<FJT>::SendGoalOptions();
  opts.result_callback =
    [&prom](const rclcpp_action::ClientGoalHandle<FJT>::WrappedResult & r) {
      prom.set_value(r.code == rclcpp_action::ResultCode::SUCCEEDED);
    };

  gripper_client_->async_send_goal(goal, opts);
  bool ok = fut.get();

  if (wait_time > 0.0) {
    std::this_thread::sleep_for(std::chrono::duration<double>(wait_time));
  }
  return ok;
}

void ArmNode::publish_auto_attach(bool enabled)
{
  std_msgs::msg::Bool msg;
  msg.data = enabled;
  auto_attach_pub_->publish(msg);
}

std::vector<double> ArmNode::joints()
{
  for (int i = 0; i < 20; i++) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!joints_.empty()) {
        return joints_;
      }
    }
    std::this_thread::sleep_for(100ms);
  }
  return {};
}

bool ArmNode::pose(ros2srrc_data::msg::Robpose & out)
{
  for (int i = 0; i < 20; i++) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (has_pose_) {
        out = pose_;
        return true;
      }
    }
    std::this_thread::sleep_for(100ms);
  }
  return false;
}

std::shared_ptr<ArmNode> start(const std::string & graspable_objects)
{
  auto node = std::make_shared<ArmNode>(graspable_objects);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  std::thread([executor]() {executor->spin();}).detach();
  return node;
}

Robmove::Goal make_robmove_goal(
  const std::string & type, double speed,
  const std::array<double, 3> & xyz, const std::array<double, 3> & ypr)
{
  Robmove::Goal goal;
  goal.type = type;
  goal.speed = static_cast<float>(speed);
  goal.x = xyz[0];
  goal.y = xyz[1];
  goal.z = xyz[2];

  // ZYX Euler convention, same as the Python HAL
  double roll = ypr[0] * M_PI / 180.0;
  double pitch = ypr[1] * M_PI / 180.0;
  double yaw = ypr[2] * M_PI / 180.0;

  double sr2 = std::sin(roll / 2), cr2 = std::cos(roll / 2);
  double sp2 = std::sin(pitch / 2), cp2 = std::cos(pitch / 2);
  double sy2 = std::sin(yaw / 2), cy2 = std::cos(yaw / 2);

  goal.qx = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
  goal.qy = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
  goal.qz = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;
  goal.qw = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
  return goal;
}

std::array<double, 3> quat_to_ypr(double qx, double qy, double qz, double qw)
{
  double roll = std::atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
  double pitch = std::asin(std::clamp(2 * (qw * qy - qz * qx), -1.0, 1.0));
  double yaw = std::atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
  return {roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI};
}

void move_abs_j(ArmNode & node, const std::vector<double> & joints, double speed, double wait_time)
{
  // movej reads just the first N fields for an N joint arm
  Move::Goal goal;
  goal.action = "MoveJ";
  goal.speed = static_cast<float>(speed);
  double * fields[] = {&goal.movej.joint1, &goal.movej.joint2, &goal.movej.joint3,
    &goal.movej.joint4, &goal.movej.joint5, &goal.movej.joint6, &goal.movej.joint7};
  for (size_t i = 0; i < joints.size() && i < 7; i++) {
    *fields[i] = joints[i];
  }

  if (!node.move_execute(goal, wait_time)) {
    RCLCPP_ERROR(rclcpp::get_logger("HAL"), "MoveAbsJ failed");
  }
}

void move_single_j(
  ArmNode & node, int joint_number, double relative_angle, double speed, double wait_time)
{
  Move::Goal goal;
  goal.action = "MoveR";
  goal.speed = static_cast<float>(speed);
  goal.mover.joint = "joint" + std::to_string(joint_number);
  goal.mover.value = relative_angle;

  if (!node.move_execute(goal, wait_time)) {
    RCLCPP_ERROR(rclcpp::get_logger("HAL"), "MoveSingleJ failed");
  }
}

void move_cartesian(
  ArmNode & node, const std::string & type, const std::array<double, 3> & xyz,
  const std::array<double, 3> & ypr, double speed, double wait_time)
{
  if (!node.robmove_execute(make_robmove_goal(type, speed, xyz, ypr), wait_time)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("HAL"), type == "LIN" ? "MoveLinear failed" : "MoveJoint failed");
  }
}

void move_rel_linear(
  ArmNode & node, const std::array<double, 3> & xyz, double speed, double wait_time)
{
  Move::Goal goal;
  goal.action = "MoveL";
  goal.speed = static_cast<float>(speed);
  goal.movel.x = xyz[0];
  goal.movel.y = xyz[1];
  goal.movel.z = xyz[2];

  if (!node.move_execute(goal, wait_time)) {
    RCLCPP_ERROR(rclcpp::get_logger("HAL"), "MoveRelLinear failed");
  }
}

void move_rel_reor(
  ArmNode & node, const std::array<double, 3> & ypr, double speed, double wait_time)
{
  Move::Goal goal;
  goal.action = "MoveROT";
  goal.speed = static_cast<float>(speed);
  // Python convention: ypr[0] -> pitch, ypr[1] -> yaw, ypr[2] -> roll
  goal.moverot.pitch = ypr[0];
  goal.moverot.yaw = ypr[1];
  goal.moverot.roll = ypr[2];

  if (!node.move_execute(goal, wait_time)) {
    RCLCPP_ERROR(rclcpp::get_logger("HAL"), "MoveRelReor failed");
  }
}

std::array<double, 3> tcp_position(ArmNode & node)
{
  ros2srrc_data::msg::Robpose pose;
  if (!node.pose(pose)) {
    RCLCPP_ERROR(rclcpp::get_logger("HAL"), "get_TCP_position: no tool pose received");
  }
  return {pose.x, pose.y, pose.z};
}

std::array<double, 3> tcp_orientation(ArmNode & node)
{
  ros2srrc_data::msg::Robpose pose;
  if (!node.pose(pose)) {
    RCLCPP_ERROR(rclcpp::get_logger("HAL"), "get_TCP_orientation: no tool pose received");
  }
  return quat_to_ypr(pose.qx, pose.qy, pose.qz, pose.qw);
}

std::vector<double> joint_states(ArmNode & node)
{
  auto values = node.joints();
  if (values.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("HAL"), "get_Joint_states: no joint state received");
  }
  return values;
}

}  // namespace pick_place::detail
