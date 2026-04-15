#include "Map.hpp"

Map::Map(std::function<Pose3D()> pose_getter, std::function<Pose3D()> noisy_pose_getter)
    : pose_getter_(pose_getter), noisy_pose_getter_(noisy_pose_getter) {}

cv::Mat Map::RTx(double angle, double tx, double ty, double tz) {
    cv::Mat RT = (cv::Mat_<double>(4, 4) <<
        1, 0, 0, tx,
        0, std::cos(angle), -std::sin(angle), ty,
        0, std::sin(angle), std::cos(angle), tz,
        0, 0, 0, 1);
    return RT;
}

cv::Mat Map::RTy(double angle, double tx, double ty, double tz) {
    cv::Mat RT = (cv::Mat_<double>(4, 4) <<
        std::cos(angle), 0, std::sin(angle), tx,
        0, 1, 0, ty,
        -std::sin(angle), 0, std::cos(angle), tz,
        0, 0, 0, 1);
    return RT;
}

cv::Mat Map::RTz(double angle, double tx, double ty, double tz) {
    cv::Mat RT = (cv::Mat_<double>(4, 4) <<
        std::cos(angle), -std::sin(angle), 0, tx,
        std::sin(angle), std::cos(angle), 0, ty,
        0, 0, 1, tz,
        0, 0, 0, 1);
    return RT;
}

cv::Mat Map::RTVacuum() {
    return RTz(M_PI / 2.0, 50, 70, 0);
}

std::vector<double> Map::getRobotCoordinates() {
    Pose3D pose = pose_getter_();
    double x = pose.x;
    double y = pose.y;

    double scale_y = -23.53;
    double offset_y = -31.95;
    y = scale_y * (offset_y - y);

    double scale_x = -23.58;
    double offset_x = -20.36;
    x = scale_x * (offset_x - x);

    return {x, y, pose.yaw};
}

std::vector<double> Map::getRobotCoordinatesWithNoise() {
    Pose3D pose = noisy_pose_getter_();
    double x = pose.x;
    double y = pose.y;

    double scale_y = -23.53;
    double offset_y = -31.95;
    y = scale_y * (offset_y - y);

    double scale_x = -23.58;
    double offset_x = -20.36;
    x = scale_x * (offset_x - x);

    return {x, y, pose.yaw};
}

void Map::reset() {
}