#ifndef DRONE_SENSORS_H
#define DRONE_SENSORS_H
//ICE
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
//INTERFACES
#include <camera.h>
#include <pose3d.h>

#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <visionlib/colorspaces/colorspacesmm.h>

namespace cat {

class DroneSensors {
public:
	DroneSensors(Ice::CommunicatorPtr ic);
	~DroneSensors();
	cv::Mat GetImage();
	//void Update();
    jderobot::Pose3DDataPtr GetPose3DData();

private:
	//QMutex mutex;
	//QMutex mutexDrone;
	//cv::Mat image;
	Ice::CommunicatorPtr ic;
	jderobot::CameraPrx cprx;
    jderobot::Pose3DPrx p3dprx;
    //jderobot::Pose3DDataPtr pose3DDataPtr;
};

} // cat

#endif // DRONE_SENSORS_H
