#include "HAL.hpp"
#include "WebGUI.hpp"
#include "academy.cpp"
#include "rclcpp/rclcpp.hpp"
#include <bits/stdc++.h>
#include <filesystem>
#include <string>
#include <thread>

void start_console()
{
  int virtual_terminal = 0;
  for (const auto &entry : std::filesystem::directory_iterator("/dev/pts/"))
  {
    std::filesystem::path outfilename = entry.path();
    std::string filename = outfilename.filename().string();
    if (filename != "ptmx" && std::stoi(filename) > virtual_terminal)
    {
      virtual_terminal = std::stoi(filename);
    }
  }

  const std::string v_terminal_str = "/dev/pts/" + std::to_string(virtual_terminal);

  if (freopen(v_terminal_str.c_str(), "w", stdout) == NULL)
  {
    std::cerr << "Error redirecting stdout!" << std::endl;
  }

  if (freopen(v_terminal_str.c_str(), "w", stderr) == NULL)
  {
    std::cerr << "Error redirecting stderr!" << std::endl;
  }

  if (freopen(v_terminal_str.c_str(), "w", stdin) == NULL)
  {
    std::cerr << "Error redirecting stdin!" << std::endl;
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  start_console();

  auto WebGUI_node = std::make_shared<WebGUI>();
  rclcpp::executors::SingleThreadedExecutor gui_executor;
  for (const auto& node : WebGUI_node->get_nodes()) {
    gui_executor.add_node(node);
  }

#ifdef USER_NODE
  auto user_node = std::make_shared<UserNode>();
  rclcpp::executors::SingleThreadedExecutor user_executor;
  user_executor.add_node(user_node);

  std::thread gui_ros([&gui_executor]{ gui_executor.spin(); });
  std::thread user_ros([&user_executor]{ user_executor.spin(); });

  gui_ros.join();
  user_ros.join();
#else
  auto HAL_node = std::make_shared<HAL>();
  rclcpp::executors::SingleThreadedExecutor hal_executor;
  hal_executor.add_node(HAL_node);

  std::thread gui_ros([&gui_executor]{ gui_executor.spin(); });
  std::thread hal_ros([&hal_executor]{ hal_executor.spin(); });
  std::thread user_api(exercise);

  user_api.join();
  gui_ros.join();
  hal_ros.join();
#endif

  rclcpp::shutdown();
  return 0;
}