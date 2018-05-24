#ifndef TRAVEL_H
#define TRAVEL_H

#include <math.h>
#include <cmdvel.h>
#include <ardroneextra.h>
#include "config/path_config.h"
#include "parser.h"
#include "quaternion.h"
#include "config/travel_config.h"
#include <pose3d.h>
//#include "drone_pose3dprx.h"
#include "common.h"

namespace mouse {

class Travel {
public:
    Travel(jderobot::Pose3DPrx pose3dprx, jderobot::CMDVelPrx,
           jderobot::ArDroneExtraPrx, std::string path_file,
           std::string xml_key);
    ~Travel();
    void StartTravel();
    bool NextMovement();
    bool GoTo(double x, double y, double z);
    bool GoTo(double x, double y);
    bool MoveToNextPoint();
    //Call before StartTravel:
    void ConfigurePath();
    void set_linear_vel(double v);
    void set_angular_vel(double v);
protected:
    jderobot::CMDVelPrx cmdvelprx;
    jderobot::ArDroneExtraPrx extraprx;
    std::string path_file_;
    std::string file_keyword_;
    TravelConfig *travel_config_;
    PathConfig *path_config_;
    jderobot::Pose3DPrx pose3dprx_;
private:
    double lin_vel_;
    double ang_vel_;
    mutable std::vector<double> beacon_;
    void PrintAndAddOffset();
};

} //mouse

#endif
