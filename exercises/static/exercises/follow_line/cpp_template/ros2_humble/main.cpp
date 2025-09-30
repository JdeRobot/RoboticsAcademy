#include "HAL.hpp"
#include "academy.cpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto user_node = std::make_shared<FollowLineNode>();
  auto HAL_node = std::make_shared<HAL>();

  rclcpp::executors::SingleThreadedExecutor executor;
  // rclcpp::executors::MultiThreadedExecutor executor(
  //     rclcpp::executor::ExecutorArgs(), 2);

  executor.add_node(HAL_node);
  executor.add_node(user_node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
