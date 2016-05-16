#ifndef SENSORS_H
#define SENSORS_H

#include <QMutex>

#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>

#include <jderobot/pose3d.h>
#include <jderobot/camera.h>
#include <jderobot/laser.h>

//Opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class Sensors
{
public:
    Sensors(Ice::CommunicatorPtr ic);

    void update();

    float getRobotPoseX();
    float getRobotPoseY();
    float getRobotPoseTheta();
    std::vector<float> getLaserData();

    cv::Mat getCamera2();
    cv::Mat getCamera1();


private:

    QMutex mutex;

    Ice::CommunicatorPtr ic;

    // ICE INTERFACES
    jderobot::Pose3DPrx p3dprx;
    jderobot::Pose3DDataPtr pose3ddata;
    jderobot::CameraPrx camera2;
    jderobot::CameraPrx camera1;

    //ICE interfaces available for connection on demand
    bool laserON;
    bool pose3dON;
    bool camera1ON;
    bool camera2ON;

    //ROBOT POSE
    double robotx;
    double roboty;
    double robottheta;

    //LASER DATA
    bool boolLaser;
    jderobot::LaserPrx laserprx;
    jderobot::LaserDataPtr ld;
    std::vector<float> laserData;

    //CAMERADATA7

    cv::Mat image1;
    cv::Mat image2;


};

#endif // SENSORS_H
