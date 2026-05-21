#include "Map.hpp"
#include <cmath>
#include <algorithm>

Map::Map(std::function<Pose3d()> pose_cb)
    : worldWidth(500), worldHeight(500), gridWidth(400), gridHeight(400),
      pose_callback_(pose_cb)
{
    grid_ = cv::Mat(gridHeight, gridWidth, CV_64FC1, cv::Scalar(0.0));
}

cv::Mat Map::RTx(double angle, double tx, double ty, double tz)
{
    return (cv::Mat_<double>(4, 4) << 
        1, 0, 0, tx,
        0, std::cos(angle), -std::sin(angle), ty,
        0, std::sin(angle), std::cos(angle), tz,
        0, 0, 0, 1);
}

cv::Mat Map::RTy(double angle, double tx, double ty, double tz)
{
    return (cv::Mat_<double>(4, 4) << 
        std::cos(angle), 0, std::sin(angle), tx,
        0, 1, 0, ty,
        -std::sin(angle), 0, std::cos(angle), tz,
        0, 0, 0, 1);
}

cv::Mat Map::RTz(double angle, double tx, double ty, double tz)
{
    return (cv::Mat_<double>(4, 4) << 
        std::cos(angle), -std::sin(angle), 0, tx,
        std::sin(angle), std::cos(angle), 0, ty,
        0, 0, 1, tz,
        0, 0, 0, 1);
}

cv::Mat Map::RTWorldGrid()
{
    cv::Mat rtx = RTx(CV_PI, 0, 0, 0);
    cv::Mat rtz = RTz(-CV_PI / 2.0, gridWidth / 2.0, -gridHeight / 2.0, 0);
    return rtx * rtz;
}

cv::Mat Map::RTGridWorld()
{
    cv::Mat rtx = RTx(-CV_PI, 0, 0, 0);
    cv::Mat rtz = RTz(CV_PI / 2.0, worldWidth / 2.0, worldHeight / 2.0, 0);
    return rtz * rtx;
}

std::vector<int> Map::worldToGrid(double worldX, double worldY)
{
    double gridX = (worldY + worldHeight / 2.0) * gridHeight / worldHeight;
    double gridY = (worldX + worldWidth / 2.0) * gridWidth / worldWidth;
    
    gridX = std::max(0.0, std::min(gridX, static_cast<double>(gridWidth)));
    gridY = std::max(0.0, std::min(gridY, static_cast<double>(gridHeight)));
    
    return {static_cast<int>(gridX), static_cast<int>(gridY)};
}

std::vector<double> Map::gridToWorld(int gridX, int gridY)
{
    double worldX = gridY * worldHeight / static_cast<double>(gridHeight) - worldHeight / 2.0;
    double worldY = gridX * worldWidth / static_cast<double>(gridWidth) - worldWidth / 2.0;
    return {worldX, worldY};
}

cv::Mat Map::RTFormula()
{
    return RTz(CV_PI / 2.0, 50, 70, 0);
}

double Map::getGridVal(int x, int y)
{
    std::lock_guard<std::mutex> lock(grid_mtx_);
    return grid_.at<double>(y, x);
}

void Map::setGridVal(int x, int y, double val)
{
    std::lock_guard<std::mutex> lock(grid_mtx_);
    grid_.at<double>(y, x) = val;
}

std::vector<int> Map::getTaxiCoordinates()
{
    Pose3d pose = pose_callback_();
    return worldToGrid(pose.x, pose.y);
}

std::vector<double> Map::getTaxiAngle()
{
    Pose3d pose = pose_callback_();
    return {pose.yaw};
}

std::vector<int> Map::rowColumn(const std::vector<double>& pose)
{
    double x = pose[0];
    double y = pose[1];

    x = x + 250.0;
    x = (x / 500.0) * 400.0;
    x = std::max(0.0, std::min(x, 400.0));
    int x_i = static_cast<int>(x);

    y = y + 250.0;
    y = (y / 500.0) * 400.0;
    y = std::max(0.0, std::min(y, 400.0));
    int y_i = static_cast<int>(y);

    return {y_i, x_i};
}

void Map::reset()
{
}

void Map::robotPose()
{
}

cv::Mat Map::getMap(const std::string& url)
{
    return cv::imread(url, cv::IMREAD_GRAYSCALE);
}