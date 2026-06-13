#include "WebGUI.hpp"
#ifndef USER_NODE
#include "HAL.hpp"
#endif
#include "academy.cpp"
#include "rclcpp/rclcpp.hpp"
#include <filesystem>
#include <thread>
#include <string>
#include <iostream>

class SystemBootstrapper {
public:
    static void init_hal() {
        HAL::init();
    }

    static void init_webgui() {
        WebGUI::init();
    }
};

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

  if (freopen(v_terminal_str.c_str(), "w", stdout) == NULL) {}
  if (freopen(v_terminal_str.c_str(), "w", stderr) == NULL) {}
  if (freopen(v_terminal_str.c_str(), "w", stdin) == NULL) {}
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  start_console();

  SystemBootstrapper::init_webgui();

#ifdef USER_NODE
  rclcpp::executors::SingleThreadedExecutor executor;
  auto user_node = std::make_shared<UserNode>();
  executor.add_node(user_node);
  executor.spin();
#else
  SystemBootstrapper::init_hal();

  std::thread user_thread(exercise);

  if (user_thread.joinable()) {
    user_thread.join();
  }
#endif

  rclcpp::shutdown();
  return 0;
}