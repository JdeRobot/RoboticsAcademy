#ifndef INCLUDE_MAP_HPP_
#define INCLUDE_MAP_HPP_

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <nlohmann/json.hpp>
#include "common_interfaces_cpp/hal/odometry.hpp"
#include "common_interfaces_cpp/hal/laser.hpp"

class Target {
public:
    Target(const std::string& id, const Pose3d& pose, bool active = false, bool reached = false);
    std::string getId() const;
    Pose3d getPose() const;
    bool isReached() const;
    void setReached(bool value);

    std::string id;
    Pose3d pose;
    bool reached;
    bool active;
};

class Map {
public:
    Map(std::function<LaserData()> laser_cb, std::function<Pose3d()> pose_cb);

    void setCar(double x, double y);
    void setObs(double x, double y);
    void setAvg(double x, double y);
    void setTargetPos(double x, double y);
    
    std::string get_json_data();
    std::shared_ptr<Target> getNextTarget();
    void reset();

    double targetx, targety;

private:
    std::vector<double> setPose(const Pose3d& pose);
    std::pair<nlohmann::json, double> setLaserValues();

    double carx, cary, obsx, obsy, avgx, avgy;
    std::vector<std::shared_ptr<Target>> targets_;
    nlohmann::json payload_;

    std::function<LaserData()> laser_callback_;
    std::function<Pose3d()> pose_callback_;
};

#endif