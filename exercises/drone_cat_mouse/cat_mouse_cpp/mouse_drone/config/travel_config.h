#ifndef TRAVEL_CONFIG_H
#define TRAVEL_CONFIG_H

#include <string>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <stdexcept>
#include "drone_config.h"

namespace mouse {

class TravelConfig : public DroneConfig {
public:
    TravelConfig();
    ~TravelConfig();
    virtual void AddInfo(const std::vector<std::string>&);
    double get_max_lin_vel();
    double get_max_ang_vel();
    double get_dist_thr();
    double get_zdist_thr();
    double get_xydist_thr();
    double get_yaw_thr();
    double get_zoffset();
    double get_roll_pitch_thr();

private:
    double max_lin_vel_;
    double dist_thr_;
    double zdist_thr_;
    double xydist_thr_;
    double yaw_thr_;
    double zoffset_;
    double max_ang_vel_;
    double roll_pitch_thr_;
};

} // mouse
#endif // TRAVEL_CONFIG_H
