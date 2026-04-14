#ifndef INCLUDE_MAP_HPP_
#define INCLUDE_MAP_HPP_

#include <vector>
#include <string>
#include <functional>
#include <opencv2/opencv.hpp>
#include "common_interfaces_cpp/hal/odometry.hpp"

class Map
{
public:
    Map(std::function<Pose3d()> pose_cb);

    cv::Mat RTx(double angle, double tx, double ty, double tz);
    cv::Mat RTy(double angle, double tx, double ty, double tz);
    cv::Mat RTz(double angle, double tx, double ty, double tz);

    cv::Mat RTWorldGrid();
    cv::Mat RTGridWorld();
    cv::Mat RTFormula();

    std::vector<int> worldToGrid(double worldX, double worldY);
    std::vector<double> gridToWorld(int gridX, int gridY);

    double getGridVal(int x, int y);
    void setGridVal(int x, int y, double val);

    std::vector<int> getTaxiCoordinates();
    std::vector<double> getTaxiAngle();

    std::vector<int> rowColumn(const std::vector<double>& pose);

    void reset();
    void robotPose();

    cv::Mat getMap(const std::string& url);

    int worldWidth;
    int worldHeight;
    int gridWidth;
    int gridHeight;

private:
    std::function<Pose3d()> pose_callback_;
    cv::Mat grid_;
};

#endif