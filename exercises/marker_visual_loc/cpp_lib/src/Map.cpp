#include "Map.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Map::Map(std::function<Pose3d()> pose_getter, std::function<Pose3d()> noisy_pose_getter)
    : pose_getter_(pose_getter), noisy_pose_getter_(noisy_pose_getter)
{
}

std::vector<std::vector<double>> Map::RTx(double angle, double tx, double ty, double tz)
{
    return {
        {1.0, 0.0, 0.0, tx},
        {0.0, std::cos(angle), -std::sin(angle), ty},
        {0.0, std::sin(angle), std::cos(angle), tz},
        {0.0, 0.0, 0.0, 1.0}
    };
}

std::vector<std::vector<double>> Map::RTy(double angle, double tx, double ty, double tz)
{
    return {
        {std::cos(angle), 0.0, std::sin(angle), tx},
        {0.0, 1.0, 0.0, ty},
        {-std::sin(angle), 0.0, std::cos(angle), tz},
        {0.0, 0.0, 0.0, 1.0}
    };
}

std::vector<std::vector<double>> Map::RTz(double angle, double tx, double ty, double tz)
{
    return {
        {std::cos(angle), -std::sin(angle), 0.0, tx},
        {std::sin(angle), std::cos(angle), 0.0, ty},
        {0.0, 0.0, 1.0, tz},
        {0.0, 0.0, 0.0, 1.0}
    };
}

std::vector<std::vector<double>> Map::RTVacuum()
{
    return RTz(M_PI / 2.0, 50.0, 70.0, 0.0);
}

std::vector<double> Map::getRobotCoordinates()
{
    Pose3d pose = pose_getter_();
    if (pose.timeStamp == 0.0 && pose.x == 0.0 && pose.y == 0.0) {
        return {};
    }

    double x = pose.x;
    double y = pose.y;

    double scale_y = -85.0;
    double offset_y = -6.88;
    y = scale_y * (offset_y - y);

    double scale_x = 83.0;
    double offset_x = 8.0;
    x = scale_x * (offset_x - x);

    return {x, y, pose.yaw};
}

std::vector<double> Map::getRobotCoordinatesWithNoise()
{
    Pose3d pose = noisy_pose_getter_();
    if (pose.timeStamp == 0.0 && pose.x == 0.0 && pose.y == 0.0) {
        return {};
    }

    double x = pose.x;
    double y = pose.y;

    double scale_y = -85.0;
    double offset_y = -6.88;
    y = scale_y * (offset_y - y);

    double scale_x = 83.0;
    double offset_x = 8.0;
    x = scale_x * (offset_x - x);

    return {x, y, pose.yaw};
}

void Map::reset()
{
    // Nothing to do, service takes care!
}