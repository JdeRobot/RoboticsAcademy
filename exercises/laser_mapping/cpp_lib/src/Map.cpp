#include "Map.hpp"
#include <cmath>

Map::Map(std::function<Pose3d()> pose_getter, std::function<Pose3d()> noisy_pose_getter)
    : pose_getter_(pose_getter), noisy_pose_getter_(noisy_pose_getter)
{
}

cv::Mat Map::rtx(double angle, double tx, double ty, double tz)
{
    return (cv::Mat_<double>(4, 4) << 
        1, 0, 0, tx,
        0, std::cos(angle), -std::sin(angle), ty,
        0, std::sin(angle), std::cos(angle), tz,
        0, 0, 0, 1);
}

cv::Mat Map::rty(double angle, double tx, double ty, double tz)
{
    return (cv::Mat_<double>(4, 4) << 
        std::cos(angle), 0, std::sin(angle), tx,
        0, 1, 0, ty,
        -std::sin(angle), 0, std::cos(angle), tz,
        0, 0, 0, 1);
}

cv::Mat Map::rtz(double angle, double tx, double ty, double tz)
{
    return (cv::Mat_<double>(4, 4) << 
        std::cos(angle), -std::sin(angle), 0, tx,
        std::sin(angle), std::cos(angle), 0, ty,
        0, 0, 1, tz,
        0, 0, 0, 1);
}

cv::Mat Map::rt_vacuum()
{
    return rtz(CV_PI / 2.0, 50, 70, 0);
}

std::tuple<double, double, double> Map::get_robot_coordinates()
{
    Pose3d pose = pose_getter_();
    
    double y = -23.53 * (-31.95 - pose.y);
    double x = -23.58 * (-20.36 - pose.x);

    return {x, y, pose.yaw};
}

std::tuple<double, double, double> Map::get_robot_coordinates_with_noise()
{
    Pose3d pose = noisy_pose_getter_();
    
    double y = -23.53 * (-31.95 - pose.y);
    double x = -23.58 * (-20.36 - pose.x);

    return {x, y, pose.yaw};
}

void Map::reset()
{
}