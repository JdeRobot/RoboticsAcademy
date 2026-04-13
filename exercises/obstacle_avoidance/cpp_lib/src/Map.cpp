#include "Map.hpp"
#include <cmath>
#include <fstream>

Target::Target(const std::string& id, const Pose3d& pose, bool active, bool reached)
    : id(id), pose(pose), reached(reached), active(active) {}

std::string Target::getId() const { return id; }
Pose3d Target::getPose() const { return pose; }
bool Target::isReached() const { return reached; }
void Target::setReached(bool value) { reached = value; }

Map::Map(std::function<LaserData()> laser_cb, std::function<Pose3d()> pose_cb)
    : carx(0), cary(0), obsx(0), obsy(0), avgx(0), avgy(0),
      targetx(0), targety(0), laser_callback_(laser_cb), pose_callback_(pose_cb) {
    
    std::ifstream file("config/targets_f1.json");
    nlohmann::json j;
    if (file.is_open() && (file >> j)) {
        if (j.contains("targets")) {
            for (const auto& t : j["targets"]) {
                Pose3d p; 
                p.x = t["x"]; 
                p.y = t["y"];
                targets_.push_back(std::make_shared<Target>(t["name"], p));
            }
        }
    }
}

void Map::setCar(double x, double y) { carx = x; cary = y; }
void Map::setObs(double x, double y) { obsx = x; obsy = y; }
void Map::setAvg(double x, double y) { avgx = x; avgy = y; }
void Map::setTargetPos(double x, double y) { targetx = x; targety = y; }

std::string Map::get_json_data() {
    payload_["pose"] = setPose(pose_callback_());
    payload_["target"] = {targetx, targety};
    payload_["car"] = {carx, cary};
    payload_["obstacle"] = {obsx, obsy};
    payload_["average"] = {avgx, avgy};
    
    auto laser_res = setLaserValues();
    payload_["laser"] = laser_res.first;
    payload_["max_range"] = laser_res.second;

    return payload_.dump();
}

std::shared_ptr<Target> Map::getNextTarget() {
    for (auto& t : targets_) {
        if (!t->isReached()) {
            setTargetPos(t->pose.x, t->pose.y);
            return t;
        }
    }
    reset();
    return targets_.empty() ? nullptr : targets_[0];
}

void Map::reset() { 
    for (auto& t : targets_) t->setReached(false); 
}

std::vector<double> Map::setPose(const Pose3d& p) { 
    return {p.x, p.y, p.yaw}; 
}

std::pair<nlohmann::json, double> Map::setLaserValues() {
    LaserData laser = laser_callback_();
    nlohmann::json points = nlohmann::json::array();
    
    double max_r = laser.values.empty() ? 0.0 : laser.maxRange * 20.0;

    if (!laser.values.empty()) {
        for (size_t i = 0; i < laser.values.size(); ++i) {
            double d = std::isinf(laser.values[i]) ? max_r : laser.values[i] * 20.0;
            double a = laser.minAngle + i * ((laser.maxAngle - laser.minAngle) / laser.values.size());
            points.push_back({d, a});
        }
    } else {
        for (int i = 0; i < 180; ++i) points.push_back({0.0, 0.0});
    }

    return {points, max_r};
}