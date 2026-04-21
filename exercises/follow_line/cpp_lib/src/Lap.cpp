#include "Lap.hpp"
#include <iomanip>
#include <sstream>
#include <cmath>

Lap::Lap(std::shared_ptr<OdometryNode> pose3d) : pose3d_(pose3d) {
    reset();
}

std::string Lap::check_threshold() {
    if (!pause_condition_) {
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
    }

    if (lap_time_.count() == 0) {
        return "0";
    }

    double total_seconds = lap_time_.count();
    int hours = static_cast<int>(total_seconds / 3600);
    int minutes = static_cast<int>((total_seconds - hours * 3600) / 60);
    double seconds = total_seconds - hours * 3600 - minutes * 60;

    std::stringstream ss;
    ss << hours << ":"
       << std::setfill('0') << std::setw(2) << minutes << ":"
       << std::setfill('0') << std::setw(2) << static_cast<int>(seconds) << "."
       << std::setfill('0') << std::setw(6) << static_cast<int>(std::round((seconds - std::floor(seconds)) * 1000000));
       
    return ss.str();
}

std::string Lap::return_lap_time() {
    return std::to_string(lap_time_.count());
}

void Lap::reset() {
    start_time_ = std::chrono::system_clock::time_point();
    lap_time_ = std::chrono::duration<double>::zero();
    lap_rest_ = true;
    buffer_ = false;
    pause_condition_ = false;
}

void Lap::pause() {
    pause_condition_ = true;
}

void Lap::unpause() {
    if (pause_condition_) {
        start_time_ = std::chrono::system_clock::now();
    }
    pause_condition_ = false;
}