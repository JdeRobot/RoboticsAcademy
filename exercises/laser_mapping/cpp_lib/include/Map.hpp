#ifndef INCLUDE_MAP_HPP_
#define INCLUDE_MAP_HPP_

#include <functional>
#include <tuple>
#include <utility>
#include <opencv2/opencv.hpp>
#include "common_interfaces_cpp/hal/odometry.hpp"

// User map canvas, pixel rows follow world x and pixel columns follow world y
namespace map_canvas
{
constexpr int HEIGHT = 970;
constexpr int WIDTH = 1500;
constexpr double SCALE_X = 23.58;  // pixels per metre along world x
constexpr double SCALE_Y = 23.53;  // pixels per metre along world y
constexpr double ORIGIN_X = -20.36;  // world x of pixel row 0
constexpr double ORIGIN_Y = -31.95;  // world y of pixel column 0

// Returns the pixel row and column of a world point
inline std::pair<double, double> world_to_pixel(double x, double y)
{
    return {SCALE_X * (x - ORIGIN_X), SCALE_Y * (y - ORIGIN_Y)};
}
}

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