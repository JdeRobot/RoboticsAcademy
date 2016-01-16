#include "travel_config.h"

namespace mouse {

TravelConfig::TravelConfig() {
    //default values:
    max_lin_vel_ = 1;
    max_ang_vel_ = 1.75;
    dist_thr_ = 0.5;
    zdist_thr_ = 0.1;
    xydist_thr_ = 0.1;
    yaw_thr_ = 0.01;
    zoffset_ = 0;
}

TravelConfig::~TravelConfig() {}

void TravelConfig::AddInfo(const std::vector<std::string>& params) {
    std::cout << "Setting travel parameters\n";
    try {
        std::istringstream(params.at(0).c_str()) >> max_lin_vel_;
        std::istringstream(params.at(1).c_str()) >> dist_thr_;
        std::istringstream(params.at(2).c_str()) >> zdist_thr_;
        std::istringstream(params.at(3).c_str()) >> xydist_thr_;
        std::istringstream(params.at(4).c_str()) >> yaw_thr_;
        std::istringstream(params.at(5).c_str()) >> zoffset_;
        std::istringstream(params.at(6).c_str()) >> max_ang_vel_;
        std::istringstream(params.at(7).c_str()) >> roll_pitch_thr_;
    } catch (std::out_of_range o) {
        std::cerr << "Not enough parameters given. "
                  << "Missing ones set to default."
                  << std::endl;
    }
    std::cout << "Configuration finished.\r\n";
}

double TravelConfig::get_max_lin_vel() {
    return max_lin_vel_;
}

double TravelConfig::get_max_ang_vel() {
    return max_ang_vel_;
}

double TravelConfig::get_dist_thr() {
    return dist_thr_;
}

double TravelConfig::get_zdist_thr() {
    return zdist_thr_;
}

double TravelConfig::get_xydist_thr() {
    return xydist_thr_;
}

double TravelConfig::get_yaw_thr() {
    return yaw_thr_;
}

double TravelConfig::get_zoffset() {
    return zoffset_;
}

double TravelConfig::get_roll_pitch_thr() {
    return roll_pitch_thr_;
}
} // mouse
