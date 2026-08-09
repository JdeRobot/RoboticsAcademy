#include "HAL.hpp"
#include "common_interfaces_cpp/hal/laser.hpp"
#include "common_interfaces_cpp/hal/motors.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"
#include "common_interfaces_cpp/hal/sim_time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <chrono>

using namespace std::chrono_literals;

std::shared_ptr<MotorsNode> HAL::motors_node_ = nullptr;
std::shared_ptr<OdometryNode> HAL::odometry_node_ = nullptr;
std::shared_ptr<LaserNode> HAL::laser_node_ = nullptr;
std::shared_ptr<SimTimeNode> HAL::sim_time_node_ = nullptr;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> HAL::executor_ =
    nullptr;
std::thread HAL::spin_thread_;
bool HAL::lift_state_ = false;

// Hidden platform control variables to avoid polluting HAL.hpp with ROS 2
// messages
static std::shared_ptr<rclcpp::Node> platform_node_ = nullptr;
static std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>>
    platform_pub_ = nullptr;

void HAL::init() {
  if (!motors_node_) {
    motors_node_ = std::make_shared<MotorsNode>("/logistic_robot/cmd_vel", 4.0,
                                                0.3, "hal_motors");
    odometry_node_ =
        std::make_shared<OdometryNode>("/logistic_robot/odom", "hal_odom");
    laser_node_ =
        std::make_shared<LaserNode>("/logistic_robot/laser/scan", "hal_laser");
    sim_time_node_ = std::make_shared<SimTimeNode>("hal_sim_time");

    // Platform control direct topic
    platform_node_ = std::make_shared<rclcpp::Node>("platform_cmd_node");
    platform_pub_ = platform_node_->create_publisher<std_msgs::msg::Float64>(
        "/logistic_robot/platform/cmd_vel", 10);

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(motors_node_);
    executor_->add_node(odometry_node_);
    executor_->add_node(laser_node_);
    executor_->add_node(sim_time_node_);
    executor_->add_node(platform_node_);

    spin_thread_ = std::thread([]() { executor_->spin(); });
    spin_thread_.detach();
  }
}

void HAL::set_v(const float velocity) {
  if (motors_node_)
    motors_node_->sendV(static_cast<double>(velocity));
}

void HAL::set_w(const float velocity) {
  if (motors_node_)
    motors_node_->sendW(static_cast<double>(velocity));
}

HAL::Pose3d HAL::get_pose3d() {
  if (!odometry_node_)
    return HAL::Pose3d{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  auto raw_pose = odometry_node_->getPose3d();
  return HAL::Pose3d{raw_pose.x,    raw_pose.y,        raw_pose.z,
                     raw_pose.h,    raw_pose.yaw,      raw_pose.pitch,
                     raw_pose.roll, raw_pose.timeStamp};
}

double HAL::get_sim_time() {
  if (!sim_time_node_)
    return 0.0;
  return sim_time_node_->getSimTime().to_double();
}

HAL::LaserData HAL::get_laser_data() {
  if (!laser_node_)
    return HAL::LaserData{};

  auto raw_laser = laser_node_->getLaserData();
  while (raw_laser.values.empty() && rclcpp::ok()) {
    std::this_thread::sleep_for(5ms);
    raw_laser = laser_node_->getLaserData();
  }

  return HAL::LaserData{raw_laser.values,   raw_laser.minAngle,
                        raw_laser.maxAngle, raw_laser.minRange,
                        raw_laser.maxRange, raw_laser.timeStamp};
}

void HAL::lift() {
  lift_state_ = true;
  if (platform_pub_) {
    std_msgs::msg::Float64 msg;
    msg.data = 5.0;
    platform_pub_->publish(msg);
  }
}

void HAL::putdown() {
  lift_state_ = false;
  if (platform_pub_) {
    std_msgs::msg::Float64 msg;
    msg.data = -5.0;
    platform_pub_->publish(msg);
  }
}

bool HAL::get_lift_state() { return lift_state_; }