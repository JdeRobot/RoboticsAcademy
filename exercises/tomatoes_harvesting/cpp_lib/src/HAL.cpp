#include "HAL.hpp"

#include "common_interfaces_cpp/hal/camera.hpp"
#include "common_interfaces_cpp/hal/laser.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2srrc_data/action/robmove.hpp"
#include "ros2srrc_data/msg/robpose.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

using namespace std::chrono_literals;

namespace
{

using FJT = control_msgs::action::FollowJointTrajectory;
using Robmove = ros2srrc_data::action::Robmove;
using Robpose = ros2srrc_data::msg::Robpose;

constexpr double MAX_V = 0.5;
constexpr double MAX_W = 0.8;
const std::vector<std::string> ARM_JOINTS = {
  "ur10_shoulder_pan_joint",
  "ur10_shoulder_lift_joint",
  "ur10_elbow_joint",
  "ur10_wrist_1_joint",
  "ur10_wrist_2_joint",
  "ur10_wrist_3_joint",
};
// Folded over the cabinet, the pose the robot starts in
const std::vector<double> ARM_HOME = {1.5708, -2.75, 2.67, -1.5708, -1.5708, 0.0};
// Unfolded in front of the base
// MoveIt sees the folded arm touching the cabinet and plans best from here
const std::vector<double> ARM_READY = {1.1428, -2.1233, 1.9619, -1.4094, -1.5708, -1.9988};
constexpr double ARM_SPEED = 0.3;
// Pilz plans the same move every time
// The harvest head joint limits keep the arm on one configuration
const std::string ARM_PLANNER = "PTP";
// Finger angles of the harvest head, the student only deals with forces
constexpr double GRIPPER_OPEN = 0.0;
constexpr double GRIPPER_CLOSED = 0.20;
// The fingers close smoothly at this rate while the grip force is watched
constexpr double GRIPPER_STEP = 0.003;
constexpr double GRIP_RATE = 20.0;
// The head spins a tomato several fast turns, the shaft has five turns of travel each way
constexpr double TWIST_SPEED = 2 * 2 * M_PI;
constexpr double TWIST_RANGE = 10 * M_PI;
const std::string CAMERA_TOPIC = "/mmo500/harvest_camera/image";
const std::string DEPTH_TOPIC = "/mmo500/harvest_camera/depth_image";
const std::string INFO_TOPIC = "/mmo500/harvest_camera/camera_info";
// Depth is only rendered while asked for and dropped after this long without use
constexpr double DEPTH_IDLE = 5.0;
constexpr double DEPTH_TIMEOUT = 3.0;

Eigen::Matrix4d transform(double x, double y, double z, double roll, double pitch, double yaw)
{
  Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
  t.block<3, 3>(0, 0) =
    (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  t.block<3, 1>(0, 3) << x, y, z;
  return t;
}

Eigen::Matrix3d quaternion_matrix(double x, double y, double z, double w)
{
  return Eigen::Quaterniond(w, x, y, z).normalized().toRotationMatrix();
}

// Grasp point to wrist camera optical frame, same numbers as mmo500_harvest_gripper.urdf.xacro
const Eigen::Matrix4d TCP_TO_OPTICAL =
  transform(0.265, 0.0, 0.09, 0.0, M_PI / 2, 0.0).inverse() *
  transform(0.08, 0.0, 0.045, M_PI, -0.1, 0.0) *
  transform(0.0, 0.0, 0.0, -M_PI / 2, 0.0, -M_PI / 2);

double seconds_since(std::chrono::steady_clock::time_point t)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count();
}

builtin_interfaces::msg::Duration to_duration(double seconds)
{
  builtin_interfaces::msg::Duration d;
  d.sec = static_cast<int32_t>(seconds);
  d.nanosec = static_cast<uint32_t>((seconds - d.sec) * 1e9);
  return d;
}

// Arm pose, head controllers and base of the MMO-500 harvester
class HarvestNode : public rclcpp::Node
{
public:
  HarvestNode()
  : rclcpp::Node("hal_harvest_node")
  {
    pose_sub_ = create_subscription<Robpose>(
      "/mmo500/Robpose", 10, [this](const Robpose::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        tcp_ = *msg;
      });
    force_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/tomato_greenhouse/grip_force", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {force_ = msg->data;});
    finger_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/mmo500/gripper_controller/joint_trajectory", 10);
    base_pub_ = create_publisher<geometry_msgs::msg::Twist>("/mmo500/cmd_vel", 10);
    for (const auto & name : {"arm_controller", "gripper_controller", "twist_controller"}) {
      controllers_[name] = rclcpp_action::create_client<FJT>(
        this, std::string("/mmo500/") + name + "/follow_joint_trajectory");
    }
    robmove_ = rclcpp_action::create_client<Robmove>(this, "/mmo500/Robmove");
  }

  std::optional<Robpose> tcp() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return tcp_;
  }

  double force() const {return force_;}

  void publish_fingers(double angle)
  {
    trajectory_msgs::msg::JointTrajectory msg;
    msg.joint_names = {"harvest_finger_1_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {angle};
    point.time_from_start = to_duration(1.0 / GRIP_RATE);
    msg.points.push_back(point);
    finger_pub_->publish(msg);
  }

  // Keeps the last twist so forward, sideways and yaw speeds keep each other
  void set_base(int axis, double value)
  {
    std::lock_guard<std::mutex> lock(base_mutex_);
    if (axis == 0) {
      twist_.linear.x = value;
    } else if (axis == 1) {
      twist_.linear.y = value;
    } else {
      twist_.angular.z = value;
    }
    base_pub_->publish(twist_);
  }

  // Sends one trajectory point and waits until the controller has reached it
  bool follow(
    const std::string & controller, const std::vector<std::string> & joints,
    const std::vector<double> & positions, double duration, bool wait = true)
  {
    auto client = controllers_.at(controller);
    if (!client->wait_for_action_server(10s)) {
      std::cout << controller << " not available" << std::endl;
      return false;
    }
    FJT::Goal goal;
    goal.trajectory.joint_names = joints;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = to_duration(duration);
    goal.trajectory.points.push_back(point);

    auto done = std::make_shared<std::promise<bool>>();
    auto resolved = std::make_shared<std::once_flag>();
    auto finish = [done, resolved](bool ok) {
        std::call_once(*resolved, [&]() {done->set_value(ok);});
      };
    auto options = rclcpp_action::Client<FJT>::SendGoalOptions();
    options.goal_response_callback =
      [finish](const rclcpp_action::ClientGoalHandle<FJT>::SharedPtr & handle) {
        if (!handle) {
          finish(false);
        }
      };
    // An aborted goal also answers, only a reached one counts
    options.result_callback =
      [finish](const rclcpp_action::ClientGoalHandle<FJT>::WrappedResult & result) {
        finish(result.result && result.result->error_code == FJT::Result::SUCCESSFUL);
      };
    auto sent = client->async_send_goal(goal, options);
    if (!wait) {
      return true;
    }
    auto future = done->get_future();
    if (sent.wait_for(10s) != std::future_status::ready) {
      return false;
    }
    const auto limit = std::chrono::duration<double>(duration + 15.0);
    return future.wait_for(limit) == std::future_status::ready && future.get();
  }

  // Plans and runs a move of the grasp point, true on success
  bool robmove(double x, double y, double z, double qx, double qy, double qz, double qw)
  {
    if (!robmove_->wait_for_action_server(10s)) {
      std::cout << "Robmove not available" << std::endl;
      return false;
    }
    Robmove::Goal goal;
    goal.type = ARM_PLANNER;
    goal.speed = ARM_SPEED;
    goal.x = x;
    goal.y = y;
    goal.z = z;
    goal.qx = qx;
    goal.qy = qy;
    goal.qz = qz;
    goal.qw = qw;

    // The promise is resolved once by the result or by a rejected goal
    auto done = std::make_shared<std::promise<std::pair<bool, std::string>>>();
    auto resolved = std::make_shared<std::once_flag>();
    auto finish = [done, resolved](bool ok, const std::string & message) {
        std::call_once(*resolved, [&]() {done->set_value({ok, message});});
      };
    auto options = rclcpp_action::Client<Robmove>::SendGoalOptions();
    options.goal_response_callback =
      [finish](const rclcpp_action::ClientGoalHandle<Robmove>::SharedPtr & handle) {
        if (!handle) {
          finish(false, "goal rejected");
        }
      };
    options.result_callback =
      [finish](const rclcpp_action::ClientGoalHandle<Robmove>::WrappedResult & result) {
        if (result.result) {
          finish(result.result->success, result.result->message);
        } else {
          finish(false, "no result");
        }
      };
    robmove_->async_send_goal(goal, options);
    const auto [ok, message] = done->get_future().get();
    std::cout << "RobMove ACTION EXECUTED -> Result: " << message << std::endl;
    return ok;
  }

private:
  rclcpp::Subscription<Robpose>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr force_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr finger_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_pub_;
  std::map<std::string, rclcpp_action::Client<FJT>::SharedPtr> controllers_;
  rclcpp_action::Client<Robmove>::SharedPtr robmove_;
  std::optional<Robpose> tcp_;
  std::atomic<double> force_{0.0};
  geometry_msgs::msg::Twist twist_;
  mutable std::mutex mutex_;
  std::mutex base_mutex_;
};

// Depth of the wrist camera in m, subscribed only while it is being asked for
class DepthOnDemand : public rclcpp::Node
{
public:
  DepthOnDemand()
  : rclcpp::Node("hal_harvest_depth"), last_use_(std::chrono::steady_clock::now() - 1h)
  {
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      INFO_TOPIC, 10, [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        k_ = msg->k;
      });
    // Shares the callback group of the depth callback
    // so the subscription never changes inside one
    timer_ = create_wall_timer(50ms, [this]() {serve();});
  }

  std::optional<std::array<double, 9>> k() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return k_;
  }

  // Depth image taken after the call, empty if none arrives in time
  cv::Mat get()
  {
    const auto asked = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_use_ = asked;
    }
    while (seconds_since(asked) < DEPTH_TIMEOUT) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!depth_.empty() && stamp_ >= asked) {
          return depth_.clone();
        }
      }
      std::this_thread::sleep_for(10ms);
    }
    return cv::Mat();
  }

private:
  void serve()
  {
    bool wanted;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      wanted = seconds_since(last_use_) < DEPTH_IDLE;
    }
    if (wanted && !depth_sub_) {
      depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        DEPTH_TOPIC, 1, [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
          cv::Mat depth = cv_bridge::toCvCopy(msg, msg->encoding)->image;
          std::lock_guard<std::mutex> lock(mutex_);
          depth_ = depth;
          stamp_ = std::chrono::steady_clock::now();
        });
    } else if (!wanted && depth_sub_) {
      depth_sub_.reset();
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<std::array<double, 9>> k_;
  cv::Mat depth_;
  std::chrono::steady_clock::time_point stamp_;
  std::chrono::steady_clock::time_point last_use_;
  mutable std::mutex mutex_;
};

// Everything the HAL talks to, created the first time any function is called
struct State
{
  std::shared_ptr<OdometryNode> odometry;
  std::shared_ptr<LaserNode> front_laser;
  std::shared_ptr<LaserNode> back_laser;
  std::shared_ptr<CameraNode> camera;
  std::shared_ptr<DepthOnDemand> depth;
  std::shared_ptr<HarvestNode> harvest;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  double finger_angle = GRIPPER_OPEN;
  double twist_angle = 0.0;
};

State & state()
{
  static State s;
  static std::once_flag once;
  std::call_once(
    once, []() {
      std::cout << "HAL MMO-500 harvester initializing" << std::endl;
      s.odometry = std::make_shared<OdometryNode>("/mmo500/odom", "hal_odometry_node");
      s.front_laser = std::make_shared<LaserNode>(
        "/mmo500/front_laser/scan", "hal_front_laser_node");
      s.back_laser = std::make_shared<LaserNode>("/mmo500/back_laser/scan", "hal_back_laser_node");
      s.camera = std::make_shared<CameraNode>(CAMERA_TOPIC, "hal_harvest_camera");
      s.depth = std::make_shared<DepthOnDemand>();
      s.harvest = std::make_shared<HarvestNode>();

      // Callbacks are served as they arrive, the grip watches a force that must never lag behind
      s.executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
      s.executor->add_node(s.odometry);
      s.executor->add_node(s.front_laser);
      s.executor->add_node(s.back_laser);
      s.executor->add_node(s.camera);
      s.executor->add_node(s.depth);
      s.executor->add_node(s.harvest);
      std::thread([]() {s.executor->spin();}).detach();
    });
  return s;
}

HAL::LaserData wait_laser(LaserNode & node)
{
  ::LaserData laser = node.getLaserData();
  while (laser.values.empty()) {
    std::this_thread::sleep_for(10ms);
    laser = node.getLaserData();
  }
  return {laser.values, laser.minAngle, laser.maxAngle, laser.minRange, laser.maxRange,
    laser.timeStamp};
}

bool set_fingers(double angle, double duration)
{
  state().finger_angle = angle;
  return state().harvest->follow(
    "gripper_controller", {"harvest_finger_1_joint"}, {angle}, duration);
}

}  // namespace

/// BASE ///

HAL::Pose3d HAL::get_pose3d()
{
  const ::Pose3d p = state().odometry->getPose3d();
  Pose3d out{};
  out.x = p.x;
  out.y = p.y;
  out.z = p.z;
  out.h = p.h;
  out.yaw = p.yaw;
  out.pitch = p.pitch;
  out.roll = p.roll;
  for (size_t i = 0; i < 4 && i < p.q.size(); ++i) {
    out.q[i] = p.q[i];
  }
  out.timeStamp = p.timeStamp;
  return out;
}

HAL::LaserData HAL::get_front_laser_data()
{
  return wait_laser(*state().front_laser);
}

HAL::LaserData HAL::get_back_laser_data()
{
  return wait_laser(*state().back_laser);
}

void HAL::set_vx(double velocity)
{
  state().harvest->set_base(0, std::clamp(velocity, -MAX_V, MAX_V));
}

void HAL::set_vy(double velocity)
{
  state().harvest->set_base(1, std::clamp(velocity, -MAX_V, MAX_V));
}

void HAL::set_w(double velocity)
{
  state().harvest->set_base(2, std::clamp(velocity, -MAX_W, MAX_W));
}

/// ARM ///

bool HAL::move_arm(double x, double y, double z, double heading)
{
  const double c = std::cos(heading), s = std::sin(heading);
  // Grasp frame with z along the fingers and x up
  Eigen::Matrix3d m;
  m << 0.0, s, c,
    0.0, -c, s,
    1.0, 0.0, 0.0;
  const double qw = std::sqrt(std::max(0.0, 1.0 + m(0, 0) + m(1, 1) + m(2, 2))) / 2;
  const double qx = std::copysign(
    std::sqrt(std::max(0.0, 1.0 + m(0, 0) - m(1, 1) - m(2, 2))) / 2, m(2, 1) - m(1, 2));
  const double qy = std::copysign(
    std::sqrt(std::max(0.0, 1.0 - m(0, 0) + m(1, 1) - m(2, 2))) / 2, m(0, 2) - m(2, 0));
  const double qz = std::copysign(
    std::sqrt(std::max(0.0, 1.0 - m(0, 0) - m(1, 1) + m(2, 2))) / 2, m(1, 0) - m(0, 1));
  auto & harvest = *state().harvest;
  if (harvest.robmove(x, y, z, qx, qy, qz, qw)) {
    return true;
  }
  // A failed plan is tried again once from the unfolded pose
  harvest.follow("arm_controller", ARM_JOINTS, ARM_READY, 3.0);
  // MoveIt plans from the current state so the arm settles first
  std::this_thread::sleep_for(2s);
  if (harvest.robmove(x, y, z, qx, qy, qz, qw)) {
    return true;
  }
  std::cout << "Arm movement FAILED, the point may be out of reach" << std::endl;
  return false;
}

std::optional<HAL::ArmPose> HAL::get_arm_pose()
{
  const auto tcp = state().harvest->tcp();
  if (!tcp) {
    return std::nullopt;
  }
  const Eigen::Vector3d axis = quaternion_matrix(tcp->qx, tcp->qy, tcp->qz, tcp->qw).col(2);
  return ArmPose{tcp->x, tcp->y, tcp->z, std::atan2(axis.y(), axis.x())};
}

bool HAL::home_arm()
{
  return state().harvest->follow("arm_controller", ARM_JOINTS, ARM_HOME, 4.0);
}

/// HARVEST HEAD ///

double HAL::grip(double force)
{
  auto & s = state();
  // Keeps closing from where the fingers are so a firmer grip never lets go first
  double angle = s.finger_angle;
  const auto period = std::chrono::duration<double>(1.0 / GRIP_RATE);
  while (angle < GRIPPER_CLOSED && s.harvest->force() < force) {
    angle = std::min(GRIPPER_CLOSED, angle + GRIPPER_STEP);
    s.harvest->publish_fingers(angle);
    std::this_thread::sleep_for(period);
  }
  s.finger_angle = angle;
  // The force follows the fingers with a short delay
  std::this_thread::sleep_for(200ms);
  return s.harvest->force();
}

bool HAL::release()
{
  auto & s = state();
  const bool opened = set_fingers(GRIPPER_OPEN, 0.4);
  if (s.twist_angle != 0.0) {
    s.harvest->follow(
      "twist_controller", {"harvest_twist_joint"}, {0.0},
      std::max(0.2, std::abs(s.twist_angle) / TWIST_SPEED), false);
    s.twist_angle = 0.0;
  }
  return opened;
}

bool HAL::twist(double turns)
{
  auto & s = state();
  const double target =
    std::clamp(s.twist_angle + 2 * M_PI * turns, -TWIST_RANGE, TWIST_RANGE);
  const double duration = std::max(0.2, std::abs(target - s.twist_angle) / TWIST_SPEED);
  s.twist_angle = target;
  return s.harvest->follow("twist_controller", {"harvest_twist_joint"}, {target}, duration);
}

/// WRIST CAMERA ///

cv::Mat HAL::get_image()
{
  auto image = state().camera->getImage();
  if (!image) {
    std::cout << "Waiting for the first image on " << CAMERA_TOPIC << std::endl;
  }
  while (!image) {
    std::this_thread::sleep_for(10ms);
    image = state().camera->getImage();
  }
  return image->data.clone();
}

cv::Mat HAL::get_depth_image()
{
  cv::Mat depth = state().depth->get();
  while (depth.empty()) {
    std::cout << "Waiting for a depth image on " << DEPTH_TOPIC << std::endl;
    depth = state().depth->get();
  }
  return depth;
}

std::optional<std::array<double, 3>> HAL::pixel_to_point(double u, double v)
{
  auto & s = state();
  const cv::Mat depth = s.depth->get();
  const auto k = s.depth->k();
  const auto tcp = s.harvest->tcp();
  if (depth.empty() || !k || !tcp) {
    return std::nullopt;
  }
  const int col = static_cast<int>(std::lround(u));
  const int row = static_cast<int>(std::lround(v));
  // Median of the finite depths around the pixel
  std::vector<float> patch;
  for (int r = std::max(0, row - 2); r < std::min(depth.rows, row + 3); ++r) {
    for (int c = std::max(0, col - 2); c < std::min(depth.cols, col + 3); ++c) {
      const float d = depth.at<float>(r, c);
      if (std::isfinite(d)) {
        patch.push_back(d);
      }
    }
  }
  if (patch.empty()) {
    return std::nullopt;
  }
  std::sort(patch.begin(), patch.end());
  const size_t n = patch.size();
  const double z = n % 2 ? patch[n / 2] : (patch[n / 2 - 1] + patch[n / 2]) / 2.0;
  const Eigen::Vector4d point(
    (col - (*k)[2]) * z / (*k)[0], (row - (*k)[5]) * z / (*k)[4], z, 1.0);
  Eigen::Matrix4d base_to_tcp = Eigen::Matrix4d::Identity();
  base_to_tcp.block<3, 3>(0, 0) = quaternion_matrix(tcp->qx, tcp->qy, tcp->qz, tcp->qw);
  base_to_tcp.block<3, 1>(0, 3) << tcp->x, tcp->y, tcp->z;
  const Eigen::Vector4d result = base_to_tcp * TCP_TO_OPTICAL * point;
  return std::array<double, 3>{result.x(), result.y(), result.z()};
}
