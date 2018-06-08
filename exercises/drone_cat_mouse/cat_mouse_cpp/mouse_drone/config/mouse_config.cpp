#include "mouse_config.h"

namespace mouse {

MouseConfig::MouseConfig() {
    //default values:
    angles_[0] = 0;
    angles_[1] = 0.1;
    angles_[2] = 0.2;
    angles_[3] = 0.3;
    linears_[0] = 0;
    linears_[1] = 0.05;
    linears_[2] = 0.2;
    linears_[3] = 1;
    times_[0] = 0;
    times_[1] = 20000000; // 20 seconds
    times_[2] = 80000000; // +1 min
    times_[3] = 110000000; // +1.5 min
}

MouseConfig::~MouseConfig() {}

void MouseConfig::AddInfo(const std::vector<std::string>& params) {
    std::cout << "Setting mouse parameters\n";
    try {
        std::istringstream(params.at(0).c_str()) >> angles_[0];
        std::istringstream(params.at(1).c_str()) >> angles_[1];
        std::istringstream(params.at(2).c_str()) >> angles_[2];
        std::istringstream(params.at(3).c_str()) >> angles_[3];
        std::istringstream(params.at(4).c_str()) >> linears_[0];
        std::istringstream(params.at(5).c_str()) >> linears_[1];
        std::istringstream(params.at(6).c_str()) >> linears_[2];
        std::istringstream(params.at(7).c_str()) >> linears_[3];
        std::istringstream(params.at(8).c_str()) >> times_[0];
        std::istringstream(params.at(9).c_str()) >> times_[1];
        std::istringstream(params.at(10).c_str()) >> times_[2];
        std::istringstream(params.at(11).c_str()) >> times_[3];
    } catch (std::out_of_range o) {
        std::cerr << "Not enough parameters given. "
                  << "Missing ones set to default."
                  << std::endl;
    }
    std::cout << "Configuration finished.\r\n";
}

long MouseConfig::get_time(int mode) {
    return times_[mode];
}

double MouseConfig::get_angle(int mode) {
    return angles_[mode];
}

double MouseConfig::get_linear(int mode) {
    return linears_[mode];
}

} // mouse
