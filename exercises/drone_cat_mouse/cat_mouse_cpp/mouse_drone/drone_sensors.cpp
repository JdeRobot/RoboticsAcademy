#include "drone_sensors.h"

namespace mouse {

DroneSensors::DroneSensors(Ice::CommunicatorPtr ic) {
	this-> ic = ic;
	Ice::PropertiesPtr prop = ic->getProperties();
	//Camera
	Ice::ObjectPrx base = ic->propertyToProxy("dronesearch.camera.Proxy");
	if (0==base)
		throw "Could not create camera proxy";
	cprx = jderobot::CameraPrx::checkedCast(base);
	if (0==cprx)
		throw "Invalid camera proxy";
    std::cout << "camera connected\n";

    //IMU
	Ice::ObjectPrx baseImu = ic->propertyToProxy("dronesearch.pose3d.Proxy");
	if (baseImu==0)
		throw "Could not create imu proxy";
    p3dprx = jderobot::Pose3DPrx::checkedCast(baseImu);
	if (p3dprx==0)
		throw "Invalid imu proxy";
    std::cout << "imu connected\n";
}

DroneSensors::~DroneSensors(){
}

/*
void DroneSensors::Update() {
	mutex.lock();
		jderobot::ImageDataPtr data = cprx->getImageData();  
		image.create(data->description->height, data->description->width, CV_8UC3); 
		memcpy((unsigned char *) image.data ,&(data->pixelData[0]), image.cols*image.rows * 3);  
	mutex.unlock();
	mutexDrone.lock();
        pose3DDataPtr = p3dprx->getPose3DData();
	mutexDrone.unlock();
}
*/

cv::Mat DroneSensors::GetImage() {
	jderobot::ImageDataPtr data = cprx->getImageData();
	cv::Mat result;
    result.create(data->description->height,
                  data->description->width,
                  CV_8UC3);
    memcpy((unsigned char *)result.data,
           &(data->pixelData[0]),
           result.cols * result.rows * 3);
    return result;
}

jderobot::Pose3DDataPtr DroneSensors::GetPose3DData() {
	jderobot::Pose3DDataPtr tmp = new jderobot::Pose3DData();
    jderobot::Pose3DDataPtr pose3DDataPtr = p3dprx->getPose3DData();
	tmp->x=pose3DDataPtr->x;
	tmp->y=pose3DDataPtr->y;
	tmp->z=pose3DDataPtr->z;
	tmp->h=pose3DDataPtr->h;
	tmp->q0=pose3DDataPtr->q0;
	tmp->q1=pose3DDataPtr->q1;
	tmp->q2=pose3DDataPtr->q2;
	tmp->q3=pose3DDataPtr->q3;
    return tmp;
}

} // mouse
