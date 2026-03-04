#include "HAL.hpp"
#include "WebGUI.hpp"
#include "academy.cpp"
#include "rclcpp/rclcpp.hpp"
#include <filesystem>
#include <string>
#include <thread>

// Redirect stdout/stderr/stdin to the latest virtual terminal so that
// print output appears in the exercise console panel.
void start_console()
{
  int virtual_terminal = 0;
  for (const auto & entry : filesystem::directory_iterator("/dev/pts/"))
  {
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
  if (freopen(v_terminal_str.c_str(), "r", stdin) == NULL)
  {
    cerr << "Error redirecting stdin!" << endl;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  hal_init();  // Creates HAL node and starts its spin thread
  start_console();

#ifdef USER_NODE
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  auto user_node = std::make_shared<UserNode>();
  executor.add_node(user_node);
  thread ros([&executor] { executor.spin(); });
#else
  thread user(exercise);
#endif

  // Runs the WebSocket GUI loop in the main thread (blocking)
  WebGUI();

#ifndef USER_NODE
  user.join();
#endif
#ifdef USER_NODE
  ros.join();
#endif

  rclcpp::shutdown();
  return 0;
}
