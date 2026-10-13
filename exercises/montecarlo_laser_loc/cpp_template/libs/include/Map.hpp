#ifndef INCLUDE_MAP_HPP_
#define INCLUDE_MAP_HPP_

#include <functional>
#include <array>
#include <opencv2/opencv.hpp>

class Map
{
public:
    explicit Map(std::function<std::array<double, 3>()> pose_cb);

    cv::Mat RTx(double angle, double tx, double ty, double tz);
    cv::Mat RTy(double angle, double tx, double ty, double tz);
    cv::Mat RTz(double angle, double tx, double ty, double tz);
    cv::Mat RTVacuum();

    std::array<double, 2> getRobotCoordinates();
    double getRobotAngle();
    
    void reset();

private:
    std::function<std::array<double, 3>()> pose_callback_;
};

#endif