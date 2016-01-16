#ifndef CAT_H
#define CAT_H

#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
//#include <remoteConfig.h>
#include <ardroneextra.h>
#include <cmdvel.h>
#include <pose3d.h>
#include "drone_sensors.h"
#include "object_detected.h"
#include "quaternion.h"
#include "cat_config.h"
#include "parser.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define ALPHA 0.6
#define VERT_X_DEADBAND 30
#define VERT_Y_DEADBAND 30
#define ANGULAR_DEADBAND 30

namespace cat {

class Cat {
public:
    Cat();
    ~Cat();
    void SeekAndDestroy();

private:
    //void Config();
    void Execute(); // Método a implementar por el alumno

    long elapsed_time_;
    Ice::CommunicatorPtr ic;
    //jderobot::remoteConfigPrx configprx;
    jderobot::CMDVelPrx cmdvelprx_;
    jderobot::ArDroneExtraPrx extraprx_;
    DroneSensors * sensors_;
    int hue_min_;
    int sat_min_;
    int val_min_;
    int hue_max_;
    int sat_max_;
    int val_max_;
    int area_min_;
    int lim_area_min_;
    int area_max_;
    int lim_area_max_;
    int edge_threshold_;
    double roll_pitch_thr_;
	ObjectDetected position,lastpos;
    double velX_;
    double velY_;
    double velZ_;
    CatConfig * config_;
};

} //cat

#endif
