#include "Map.hpp"
#include <cmath>

Map::Map(std::function<Pose3d()> pose_cb)
    : pose_callback_(pose_cb)
{
}

cv::Mat Map::RTx(double angle, double tx, double ty, double tz)
{
    return (cv::Mat_<double>(4, 4) << 
        1.0, 0.0, 0.0, tx,
        0.0, std::cos(angle), -std::sin(angle), ty,
        0.0, std::sin(angle), std::cos(angle), tz,
        0.0, 0.0, 0.0, 1.0);
}

cv::Mat Map::RTy(double angle, double tx, double ty, double tz)
{
    return (cv::Mat_<double>(4, 4) << 
        std::cos(angle), 0.0, std::sin(angle), tx,
        0.0, 1.0, 0.0, ty,
        -std::sin(angle), 0.0, std::cos(angle), tz,
        0.0, 0.0, 0.0, 1.0);
}

cv::Mat Map::RTz(double angle, double tx, double ty, double tz)
{
    return (cv::Mat_<double>(4, 4) << 
        std::cos(angle), -std::sin(angle), 0.0, tx,
        std::sin(angle), std::cos(angle), 0.0, ty,
        0.0, 0.0, 1.0, tz,
        0.0, 0.0, 0.0, 1.0);
}

cv::Mat Map::RTVacuum()
{
    return RTz(CV_PI / 2.0, 50.0, 70.0, 0.0);
}

std::tuple<double, double> Map::getRobotCoordinates()
{
    auto pose = pose_callback_();
    
    double x = pose.x;
    double y = pose.y;

    x = (6.8 - x) * 20.22 * 0.545;
    y = (10.31 - y) * 20.17 * 0.72;

    // The original python explicitly returns (y, x)
    return {y, x};
}

double Map::getRobotAngle()
{
    auto pose = pose_callback_();
    return pose.yaw;
}

void Map::reset()
{
    // Nothing to do, service takes care!
}