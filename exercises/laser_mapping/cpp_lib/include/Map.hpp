#ifndef MAP_HPP_
#define MAP_HPP_

#include <functional>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

struct Pose3D {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

class Map {
public:
    Map(std::function<Pose3D()> pose_getter, std::function<Pose3D()> noisy_pose_getter);

    cv::Mat RTx(double angle, double tx, double ty, double tz);
    cv::Mat RTy(double angle, double tx, double ty, double tz);
    cv::Mat RTz(double angle, double tx, double ty, double tz);
    cv::Mat RTVacuum();

    std::vector<double> getRobotCoordinates();
    std::vector<double> getRobotCoordinatesWithNoise();
    void reset();

private:
    std::function<Pose3D()> pose_getter_;
    std::function<Pose3D()> noisy_pose_getter_;
};

#endif