#ifndef MOUSE_H
#define MOUSE_H

#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include <remoteConfig.h>
#include <ardroneextra.h>
#include <cmdvel.h>
#include <pose3d.h>
#include "travel.h"
#include "drone_sensors.h"
#include "config/mouse_config.h"

namespace mouse {

class Mouse {
public:
    Mouse();
    ~Mouse();
    void Survive();

private:
    void CheckTimesAndRunFaster();
    void SetRunningMode();

    enum {Preparing, Easy, Medium, Hard, End};
    double linears_[4]; // meters / second
    double angles_[4]; // radians / second
    double current_angle_;
    long times_[4]; // useconds
    long next_time_;
    int status_;
    long elapsed_time_;
    Ice::CommunicatorPtr ic;
    Travel *travel_;
    jderobot::remoteConfigPrx configprx;
    MouseConfig * mouse_config_;
};

} //mouse

#endif
