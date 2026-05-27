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
    return rtz(CV_PI / 2.0, 50.0, 70.0, 0.0);
}

std::optional<std::tuple<double, double, double>> Map::get_robot_coordinates()
{
    Pose3d pose = pose_getter_();
    
    if (pose.timeStamp == 0.0) {
        return std::nullopt;
    }

    double scale_y = -85.0;
    double offset_y = -6.88;
    double y = scale_y * (offset_y - pose.y);

    double scale_x = 83.0;
    double offset_x = 8.0;
    double x = scale_x * (offset_x - pose.x);

    return std::make_tuple(x, y, pose.yaw);
}

std::optional<std::tuple<double, double, double>> Map::get_robot_coordinates_with_noise()
{
    Pose3d pose = noisy_pose_getter_();
    
    if (pose.timeStamp == 0.0) {
        return std::nullopt;
    }

    double scale_y = -85.0;
    double offset_y = -6.88;
    double y = scale_y * (offset_y - pose.y);

    double scale_x = 83.0;
    double offset_x = 8.0;
    double x = scale_x * (offset_x - pose.x);

    return std::make_tuple(x, y, pose.yaw);
}

void Map::reset()
{
}