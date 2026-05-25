#ifndef INCLUDE_MAP_HPP_
#define INCLUDE_MAP_HPP_

#include <functional>
#include <array>
#include <opencv2/opencv.hpp>
#include "common_interfaces_cpp/hal/odometry.hpp"

class Map
{
public:
    explicit Map(std::function<Pose3d()> pose3d);

    cv::Mat RTx(double angle, double tx, double ty, double tz);
    cv::Mat RTy(double angle, double tx, double ty, double tz);
    cv::Mat RTz(double angle, double tx, double ty, double tz);
    cv::Mat RTVacuum();

    std::array<double, 2> getRobotCoordinates();
    double getRobotAngle();
    
    // Function to reset
    void reset();

private:
    std::function<Pose3d()> pose3d_;
};

#endif