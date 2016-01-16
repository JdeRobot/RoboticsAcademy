#ifndef DRONE_SENSORS_H
#define DRONE_SENSORS_H
//ICE
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
//INTERFACES
#include <jderobot/camera.h>
#include <jderobot/pose3d.h>

#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace mouse {

class DroneSensors {
public:
	DroneSensors(Ice::CommunicatorPtr ic);
	~DroneSensors();
	cv::Mat GetImage();
    jderobot::Pose3DDataPtr GetPose3DData();

private:
	Ice::CommunicatorPtr ic;
	jderobot::CameraPrx cprx;
    jderobot::Pose3DPrx p3dprx;
};

} // mouse

#endif // DRONE_SENSORS_H
