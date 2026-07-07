#ifndef LAP_HPP_
#define LAP_HPP_

#include "common_interfaces_cpp/hal/odometry.hpp"
#include <chrono>
#include <csignal>
#include <memory>
#include <mutex>
#include <string>

class Lap {
public:
  explicit Lap(std::shared_ptr<OdometryNode> pose3d);
  std::string check_threshold();
  std::string return_lap_time();
  void reset();
  void pause();
  void unpause();

  void myHandler(int signum) { start_time_ = std::chrono::system_clock::now(); }
  static void static_myHandler(int signum) { instance.myHandler(signum); }

private:
  std::shared_ptr<OdometryNode> pose3d_;
  std::chrono::system_clock::time_point start_time_;
  std::chrono::duration<double> lap_time_;
  bool lap_rest_;
  bool buffer_;
  bool pause_condition_;
  std::mutex lap_mutex_;
  static Lap instance;
};

#endif