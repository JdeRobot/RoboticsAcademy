#include "HAL.hpp"

using namespace std::chrono_literals;

std::shared_ptr<MotorsNode> HAL::motors_node_ = nullptr;
std::shared_ptr<LaserNode> HAL::laser_node_ = nullptr;
std::shared_ptr<BumperNode> HAL::bumper_node_ = nullptr;

HAL::HAL() : Node("hal_node")
{
  std::vector<std::string> bumper_topics = {
      "/roombaROS/events/right_bumper",
      "/roombaROS/events/center_bumper",
      "/roombaROS/events/left_bumper"
  };

  motors_node_ = std::make_shared<MotorsNode>("/cmd_vel", 1.0, 1.0);
  laser_node_ = std::make_shared<LaserNode>("/roombaROS/laser/scan");
  bumper_node_ = std::make_shared<BumperNode>(bumper_topics);

  spin_thread_ = std::thread([]() {
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(HAL::motors_node_);
      executor.add_node(HAL::laser_node_);
      executor.add_node(HAL::bumper_node_);
      
      while (rclcpp::ok()) {
          executor.spin_some();
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
  });
  spin_thread_.detach();
}

void HAL::set_v(const float speed)
{
  if (motors_node_) motors_node_->sendV(static_cast<double>(speed));
}

void HAL::set_w(const float speed)
{
  if (motors_node_) motors_node_->sendW(static_cast<double>(speed));
}

const LaserData *HAL::get_laser_data()
{
  if (!laser_node_) return nullptr;
  return new LaserData(laser_node_->getLaserData());
}

std::vector<bool> HAL::get_bumper_data()
{
  std::vector<bool> v = {false, false, false};
  if (bumper_node_) {
      BumperData b_data = bumper_node_->getBumperData();
      if (b_data.state == 1 && b_data.bumper >= 0 && b_data.bumper < 3) {
          v[b_data.bumper] = true;
      }
  }
  return v;
}