#ifndef CAT_CONFIG_H
#define CAT_CONFIG_H

//#include "drone_config.h"
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <stdexcept>
#include <sstream>

namespace cat {

class CatConfig {// : public DroneConfig {
public:
    CatConfig();
    ~CatConfig();

    void AddInfo(const std::vector<std::string>&);
    int GetMinHue();
    int GetMaxHue();
    int GetMinVal();
    int GetMaxVal();
    int GetMinSat();
    int GetMaxSat();
    int GetMaxArea();
    int GetMinArea();
    int GetMaxAreaLimit();
    int GetMinAreaLimit();
    double GetVelX();
    double GetVelY();
    double GetVelZ();
    double GetAngularVel();

private:
    int min_hue_;
    int max_hue_;
    int min_val_;
    int max_val_;
    int min_sat_;
    int max_sat_;
    int min_area_;
    int max_area_;
    int min_area_limit_;
    int max_area_limit_;
    double velX_;
    double velY_;
    double velZ_;
    double angular_vel_;
};

} // cat

#endif //FILTER_CONFIG_H
