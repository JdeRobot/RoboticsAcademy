#include "HAL.hpp"

using namespace std::chrono_literals;

std::shared_ptr<MotorsNode> HAL::motors_node_ = nullptr;
std::shared_ptr<LaserNode> HAL::laser_node_ = nullptr;
std::shared_ptr<OdometryNode> HAL::odometry_node_ = nullptr;
std::shared_ptr<OdometryNode> HAL::noisy_odometry_node_ = nullptr;

HAL::HAL() : Node("hal_node")
{
  motors_node_ = std::make_shared<MotorsNode>("/turtlebot3/cmd_vel", 4.0, 0.3);
  laser_node_ = std::make_shared<LaserNode>("/turtlebot3/laser/scan");
  odometry_node_ = std::make_shared<OdometryNode>("/turtlebot3/odom");
  noisy_odometry_node_ = std::make_shared<OdometryNode>("/turtlebot3/odom_noisy", "noisy_odometry_node");

  spin_thread_ = std::thread([]() {
      rclcpp::executors::MultiThreadedExecutor executor;
      executor.add_node(HAL::motors_node_);
      executor.add_node(HAL::laser_node_);
      executor.add_node(HAL::odometry_node_);
      executor.add_node(HAL::noisy_odometry_node_);
      
      while (rclcpp::ok()) {
          executor.spin_some();
          std::this_thread::sleep_for(std::chrono::milliseconds(11));
      }
  });
  spin_thread_.detach();
}

void HAL::setV(const float speed)
{
  if (motors_node_) {
      motors_node_->sendV(static_cast<double>(speed));
  }
}

void HAL::setW(const float speed)
{
  if (motors_node_) {
      motors_node_->sendW(static_cast<double>(speed));
  }
}

const LaserData *HAL::getLaserData()
{
  if (!laser_node_) {
      return nullptr;
  }
  
  LaserData data = laser_node_->getLaserData();
  while (data.values.empty() && rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(11));
      data = laser_node_->getLaserData();
  }
  return new LaserData(data);
}

const Pose3d *HAL::getPose3d()
{
  if (!odometry_node_) {
      return nullptr;
  }
  return new Pose3d(odometry_node_->getPose3d());
}

const Pose3d *HAL::getOdom()
{
  if (!noisy_odometry_node_) {
      return nullptr;
  }
  return new Pose3d(noisy_odometry_node_->getPose3d());
}