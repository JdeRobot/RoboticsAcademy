#ifndef INCLUDE_MAP_HPP_
#define INCLUDE_MAP_HPP_

#include <vector>
#include <functional>
#include <cmath>
#include "common_interfaces_cpp/hal/odometry.hpp"

class Map
{
public:
    Map(std::function<Pose3d()> pose_getter, std::function<Pose3d()> noisy_pose_getter);

    std::vector<std::vector<double>> RTx(double angle, double tx, double ty, double tz);
    std::vector<std::vector<double>> RTy(double angle, double tx, double ty, double tz);
    std::vector<std::vector<double>> RTz(double angle, double tx, double ty, double tz);
    std::vector<std::vector<double>> RTVacuum();

    std::vector<double> getRobotCoordinates();
    std::vector<double> getRobotCoordinatesWithNoise();
    
    void reset();

private:
    std::function<Pose3d()> pose_getter_;
    std::function<Pose3d()> noisy_pose_getter_;
};

#endif