#ifndef MOUSE_CONFIG_H
#define MOUSE_CONFIG_H

#include <string>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <stdexcept>
#include "drone_config.h"

namespace mouse {

class MouseConfig : public DroneConfig {
public:
    MouseConfig();
    ~MouseConfig();
    virtual void AddInfo(const std::vector<std::string>&);
    double get_angle(int);
    long get_time(int);
    double get_linear(int);

private:
    long times_[4];
    double angles_[4];
    double linears_[4];
};

} // mouse
#endif // MOUSE_CONFIG_H
