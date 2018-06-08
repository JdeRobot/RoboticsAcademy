#include "cat_config.h"

namespace cat {

CatConfig::CatConfig() {
}

CatConfig::~CatConfig() {
}

void CatConfig::AddInfo(const std::vector<std::string>& params) {
    std::cout << "Setting cat parameters\n";
    try {
        min_hue_ = atoi(params.at(0).c_str());
        max_hue_ = atoi(params.at(1).c_str());
        min_val_ = atoi(params.at(2).c_str());
        max_val_ = atoi(params.at(3).c_str());
        min_sat_ = atoi(params.at(4).c_str());
        max_sat_ = atoi(params.at(5).c_str());
        min_area_ = atoi(params.at(6).c_str());
        max_area_ = atoi(params.at(7).c_str());
        min_area_limit_ = atoi(params.at(8).c_str());
        max_area_limit_ = atoi(params.at(9).c_str());
        std::istringstream(params.at(10).c_str()) >> velX_;
        std::istringstream(params.at(11).c_str()) >> velY_;
        std::istringstream(params.at(12).c_str()) >> velZ_;
        std::istringstream(params.at(13).c_str()) >> angular_vel_;
    } catch (std::out_of_range o) {
        std::cerr << "Not enough parameters given. "
                  << "Missing ones set to default."
                  << std::endl;
    }
    std::cout << "Cat configuration finished.\r\n";
}

int CatConfig::GetMinHue() {
    return min_hue_;
}

int CatConfig::GetMaxHue() {
    return max_hue_;
}

int CatConfig::GetMinVal() {
    return min_val_;
}

int CatConfig::GetMaxVal() {
    return max_val_;
}

int CatConfig::GetMinSat() {
    return min_sat_;
}

int CatConfig::GetMaxSat() {
    return max_sat_;
}

int CatConfig::GetMinArea() {
    return min_area_;
}

int CatConfig::GetMaxArea() {
    return max_area_;
}

int CatConfig::GetMinAreaLimit() {
    return min_area_limit_;
}

int CatConfig::GetMaxAreaLimit() {
    return max_area_limit_;
}

double CatConfig::GetVelX() {
    return velX_;
}

double CatConfig::GetVelY() {
    return velY_;
}

double CatConfig::GetVelZ() {
    return velZ_;
}

double CatConfig::GetAngularVel() {
    return angular_vel_;
}

} // cat
