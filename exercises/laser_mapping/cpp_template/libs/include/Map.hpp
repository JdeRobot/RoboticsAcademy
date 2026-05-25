#ifndef INCLUDE_MAP_HPP_
#define INCLUDE_MAP_HPP_

#include <functional>
#include <tuple>
#include <opencv2/opencv.hpp>
#include "common_interfaces_cpp/hal/odometry.hpp"

class Map
{
public:
    Map(std::function<Pose3d()> pose_getter, std::function<Pose3d()> noisy_pose_getter);

    cv::Mat rtx(double angle, double tx, double ty, double tz);
    cv::Mat rty(double angle, double tx, double ty, double tz);
    cv::Mat rtz(double angle, double tx, double ty, double tz);
    cv::Mat rt_vacuum();

    std::tuple<double, double, double> get_robot_coordinates();
    std::tuple<double, double, double> get_robot_coordinates_with_noise();

    void reset();

private:
    std::function<Pose3d()> pose_getter_;
    std::function<Pose3d()> noisy_pose_getter_;
};

#endif