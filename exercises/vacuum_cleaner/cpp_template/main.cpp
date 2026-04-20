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
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  start_console();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);

  auto HAL_node = std::make_shared<HAL>();
  executor.add_node(HAL_node);

  auto WebGUI_node = std::make_shared<WebGUI>();

  for (const auto& node : WebGUI_node->get_nodes()) {
    executor.add_node(node);
  }

#ifdef USER_NODE
  auto user_node = std::make_shared<UserNode>();
  executor.add_node(user_node);
#else
  std::thread user(exercise);
#endif
  std::thread ros([&executor]{executor.spin();});

#ifndef USER_NODE
  user.join();
#endif
  ros.join();

  rclcpp::shutdown();
  return 0;
}