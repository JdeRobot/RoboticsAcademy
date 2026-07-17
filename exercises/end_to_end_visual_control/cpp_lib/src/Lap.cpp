#include "Lap.hpp"
#include <cmath>
#include <csignal>
#include <iomanip>
#include <sstream>

static volatile std::sig_atomic_t flag;

void set_flag(int signal) { flag = 1; }

Lap::Lap(std::shared_ptr<OdometryNode> pose3d) : pose3d_(pose3d) {
  reset();
  signal(SIGCONT, set_flag);
}

std::string Lap::check_threshold() {
  std::lock_guard<std::mutex> lock(lap_mutex_);

  if (flag == 1) {
    flag = 0;
    start_time_ = std::chrono::system_clock::now();
  }

  if (start_time_.time_since_epoch().count() != 0 && !lap_rest_) {
    auto now = std::chrono::system_clock::now();
    if (lap_time_.count() == 0) {
      lap_time_ = now - start_time_;
    } else {
      lap_time_ += now - start_time_;
    }
    start_time_ = now;
  }

  if (start_time_.time_since_epoch().count() == 0 && lap_rest_) {
    start_time_ = std::chrono::system_clock::now();
    lap_rest_ = false;
  }

  if (lap_time_.count() == 0) {
    return "0";
  }

  double total_seconds = lap_time_.count();
  int hours = static_cast<int>(total_seconds / 3600);
  int minutes = static_cast<int>((total_seconds - hours * 3600) / 60);
  double seconds = total_seconds - hours * 3600 - minutes * 60;

  std::stringstream ss;
  ss << hours << ":" << std::setfill('0') << std::setw(2) << minutes << ":"
     << std::setfill('0') << std::setw(2) << static_cast<int>(seconds) << "."
     << std::setfill('0') << std::setw(6)
     << static_cast<int>(std::round((seconds - std::floor(seconds)) * 1000000));

  return ss.str();
}

std::string Lap::return_lap_time() {
  std::lock_guard<std::mutex> lock(lap_mutex_);
  return std::to_string(lap_time_.count());
}

void Lap::reset() {
  std::lock_guard<std::mutex> lock(lap_mutex_);
  start_time_ = std::chrono::system_clock::time_point();
  lap_time_ = std::chrono::duration<double>::zero();
  lap_rest_ = true;
  buffer_ = false;
}