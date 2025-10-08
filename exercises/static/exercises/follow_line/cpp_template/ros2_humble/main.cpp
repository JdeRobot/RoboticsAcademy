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
  for (const auto &entry : filesystem::directory_iterator("/dev/pts/"))
  {
    // Converting the path to const char * in the subsequent lines
    filesystem::path outfilename = entry.path();
    string filename = outfilename.filename().string();
    if (filename != "ptmx" && stoi(filename) > virtual_terminal)
    {
      virtual_terminal = stoi(filename);
    }
  }

  const string v_terminal_str = "/dev/pts/" + to_string(virtual_terminal);

  if (freopen(v_terminal_str.c_str(), "w", stdout) == NULL)
  {
    cerr << "Error redirecting stdout!" << endl;
  }

  if (freopen(v_terminal_str.c_str(), "w", stderr) == NULL)
  {
    cerr << "Error redirecting stderr!" << endl;
  }

  if (freopen(v_terminal_str.c_str(), "w", stdin) == NULL)
  {
    cerr << "Error redirecting stdin!" << endl;
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  start_console();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

  auto HAL_node = std::make_shared<HAL>();
  executor.add_node(HAL_node);

#ifdef FollowLineNode
  auto user_node = std::make_shared<FollowLineNode>();
  executor.add_node(user_node);
#else
  thread user(exercise);
#endif
  thread ros([&executor]{executor.spin();});
  WebGUI();

  user.join();
  ros.join();

  rclcpp::shutdown();
  return 0;
}