#ifndef DRONE_CONFIG_H
#define DRONE_CONFIG_H

#include <stdlib.h>
#include <iostream>
#include <vector>

namespace mouse {

class DroneConfig {
public:
    virtual void AddInfo(const std::vector<std::string>&) = 0;
};

} //mouse

#endif
