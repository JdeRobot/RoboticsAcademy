#ifndef INCLUDE_MAP_HPP_
#define INCLUDE_MAP_HPP_

#include <functional>
#include <vector>
#include <string>
#include <utility>
#include <opencv2/opencv.hpp>
#include "common_interfaces_cpp/hal/laser.hpp"

class Map
{
public:
    Map(std::function<LaserData()> laser_f_cb,
        std::function<LaserData()> laser_r_cb,
        std::function<LaserData()> laser_b_cb);

    std::string get_json_data();

    cv::Mat RTx(double angle, double tx, double ty, double tz);
    cv::Mat RTy(double angle, double tx, double ty, double tz);
    cv::Mat RTz(double angle, double tx, double ty, double tz);

    struct LaserValues {
        std::vector<std::vector<std::pair<double, double>>> lasers;
        std::vector<double> ranges;
    };

    LaserValues setLaserValues();
    std::vector<std::pair<double, double>> setSingleLaserValue(const LaserData& laser);

private:
    std::function<LaserData()> laser_f_cb_;
    std::function<LaserData()> laser_r_cb_;
    std::function<LaserData()> laser_b_cb_;
};

#endif