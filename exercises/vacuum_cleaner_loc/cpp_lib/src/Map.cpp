#include "Map.hpp"
#include <cmath>

Map::Map(std::function<Pose3d()> pose_cb)
    : pose_callback_(pose_cb)
{
}

cv::Mat Map::RTx(double angle, double tx, double ty, double tz)
{
    return (cv::Mat_<double>(4, 4) << 
        1, 0, 0, tx,
        0, std::cos(angle), -std::sin(angle), ty,
        0, std::sin(angle), std::cos(angle), tz,
        0, 0, 0, 1);
}

cv::Mat Map::RTy(double angle, double tx, double ty, double tz)
{
    return (cv::Mat_<double>(4, 4) << 
        std::cos(angle), 0, std::sin(angle), tx,
        0, 1, 0, ty,
        -std::sin(angle), 0, std::cos(angle), tz,
        0, 0, 0, 1);
}

cv::Mat Map::RTz(double angle, double tx, double ty, double tz)
{
    return (cv::Mat_<double>(4, 4) << 
        std::cos(angle), -std::sin(angle), 0, tx,
        std::sin(angle), std::cos(angle), 0, ty,
        0, 0, 1, tz,
        0, 0, 0, 1);
}

cv::Mat Map::RTVacuum()
{
    return RTz(CV_PI / 2.0, 50.0, 70.0, 0.0);
}

std::tuple<double, double> Map::getRobotCoordinates()
{
    Pose3d pose = pose_callback_();

    double scale_y = 15.0;
    double offset_y = 63.0;
    double y = scale_y * pose.y + offset_y;

    double scale_x = -30.0;
    double offset_x = 171.0;
    double x = scale_x * pose.x + offset_x;

    return std::make_tuple(x, y);
}

double Map::getRobotAngle()
{
    Pose3d pose = pose_callback_();
    
    return pose.yaw;
}

// Function to reset
void Map::reset()
{
    // Nothing to do, service takes care!
}