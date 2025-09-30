#include "HAL.hpp"
#include "WebGUI.hpp"
#include "academy.cpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;
  // rclcpp::executors::MultiThreadedExecutor executor(
  //     rclcpp::executor::ExecutorArgs(), 2);

  auto HAL_node = std::make_shared<HAL>();
  executor.add_node(HAL_node);

  auto webGUI = WebGUI();

  #ifdef FollowLineNode
    auto user_node = std::make_shared<FollowLineNode>();
    executor.add_node(user_node);
  #else
    exercise();
  #endif

  executor.spin();

  rclcpp::shutdown();
  return 0;
}


