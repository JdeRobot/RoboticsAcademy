#ifndef INCLUDE_MAP_HPP_
#define INCLUDE_MAP_HPP_

#include <functional>
#include <tuple>
#include <opencv2/opencv.hpp>
#include "common_interfaces_cpp/hal/odometry.hpp" // For Pose3d definition

class Map
{
public:
    explicit Map(std::function<Pose3d()> pose_cb);

    cv::Mat RTx(double angle, double tx, double ty, double tz);
    cv::Mat RTy(double angle, double tx, double ty, double tz);
    cv::Mat RTz(double angle, double tx, double ty, double tz);
    cv::Mat RTVacuum();

    // Returns {y, x} based on the original Python logic
    std::tuple<double, double> getRobotCoordinates();
    double getRobotAngle();
    
    void reset();

private:
    std::function<Pose3d()> pose_callback_;
};

#endif