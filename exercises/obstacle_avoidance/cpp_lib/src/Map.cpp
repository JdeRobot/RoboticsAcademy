#include "Map.hpp"
#include <fstream>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

Target::Target(std::string id, Pose3d pose, bool active, bool reached)
    : id_(id), pose_(pose), active_(active), reached_(reached) {}

std::string Target::get_id() const { return id_; }
Pose3d Target::get_pose() const { return pose_; }
bool Target::is_reached() const { return reached_; }
void Target::set_reached(bool value) { reached_ = value; }
bool Target::is_active() const { return active_; }
void Target::set_active(bool value) { active_ = value; }

Map::Map(std::shared_ptr<LaserNode> laser, std::shared_ptr<OdometryNode> odom)
    : laser_(laser), odom_(odom),
      carx_(2.0), cary_(0.0), obsx_(0.0), obsy_(2.0),
      avgx_(-2.0), avgy_(0.0), targetx_(0.0), targety_(0.0),
      laser_running_(true)
{
    load_targets();

    laser_thread_ = std::thread(&Map::laser_poll_loop, this);
}

Map::~Map()
{
    laser_running_.store(false);
    if (laser_thread_.joinable())
        laser_thread_.join();
}

void Map::laser_poll_loop()
{
    while (laser_running_.load()) {
        LaserData data = laser_->getLaserData();
        if (!data.values.empty()) {
            std::lock_guard<std::mutex> lk(laser_mtx_);
            cached_laser_ = data;
        }
        std::this_thread::sleep_for(33ms); // ~30 Hz
    }
}

void Map::set_car(double newx, double newy) {
    std::lock_guard<std::mutex> lock(map_mtx_);
    carx_ = newx; cary_ = newy;
}

void Map::set_obs(double newx, double newy) {
    std::lock_guard<std::mutex> lock(map_mtx_);
    obsx_ = newx; obsy_ = newy;
}

void Map::set_avg(double newx, double newy) {
    std::lock_guard<std::mutex> lock(map_mtx_);
    avgx_ = newx; avgy_ = newy;
}

void Map::set_target_pos(double newx, double newy) {
    std::lock_guard<std::mutex> lock(map_mtx_);
    targetx_ = newx; targety_ = newy;
}

void Map::set_target_x(double x) {
    std::lock_guard<std::mutex> lock(map_mtx_);
    targetx_ = x;
}

void Map::set_target_y(double y) {
    std::lock_guard<std::mutex> lock(map_mtx_);
    targety_ = y;
}

std::array<double, 2> Map::get_next_target() {
    std::lock_guard<std::mutex> lock(map_mtx_);
    for (auto& target : targets_) {
        if (!target->is_reached()) {
            targetx_ = target->get_pose().x;
            targety_ = target->get_pose().y;
            return {targetx_, targety_};
        }
    }

    for (auto& target : targets_)
        target->set_reached(false);

    if (!targets_.empty()) {
        targetx_ = targets_.front()->get_pose().x;
        targety_ = targets_.front()->get_pose().y;
    }
    return {targetx_, targety_};
}

void Map::mark_current_target_reached() {
    std::lock_guard<std::mutex> lock(map_mtx_);
    for (auto& target : targets_) {
        if (!target->is_reached()) {
            target->set_reached(true);
            break;
        }
    }
}

void Map::reset() {
    std::lock_guard<std::mutex> lock(map_mtx_);
    for (auto& target : targets_)
        target->set_reached(false);
}

void Map::load_targets() {
    std::ifstream file("/resources/exercises/obstacle_avoidance/simple_circuit_targets.json");
    if (file.is_open()) {
        nlohmann::json data;
        file >> data;
        for (const auto& t : data["targets"]) {
            Pose3d p;
            p.x = t["x"];
            p.y = t["y"];
            targets_.push_back(std::make_shared<Target>(t["name"], p));
        }
    }
}

nlohmann::json Map::get_json_data() {
    LaserData laser_data;
    {
        std::lock_guard<std::mutex> lk(laser_mtx_);
        laser_data = cached_laser_;
    }
    Pose3d pose = odom_->getPose3d();

    std::lock_guard<std::mutex> lock(map_mtx_);

    nlohmann::json payload;
    payload["pose"]     = {pose.x, pose.y, pose.yaw};
    payload["target"]   = {targetx_, targety_};
    payload["car"]      = {carx_, cary_};
    payload["obstacle"] = {obsx_, obsy_};
    payload["average"]  = {avgx_, avgy_};

    constexpr int angle_limit = 180;
    std::vector<std::array<double, 2>> laser_vals;
    laser_vals.reserve(angle_limit);

    if (!laser_data.values.empty()) {
        for (int i = 0; i < angle_limit && i < static_cast<int>(laser_data.values.size()); ++i) {
            double dist = 20.0 * laser_data.values[i];
            if (std::isinf(dist) || std::isnan(dist))
                dist = 20.0 * laser_data.maxRange;
            laser_vals.push_back({dist, i * M_PI / 180.0});
        }
    } else {
        for (int i = 0; i < angle_limit; ++i)
            laser_vals.push_back({0.0, 0.0});
    }

    payload["laser"]     = laser_vals;
    payload["max_range"] = 20.0 * (laser_data.values.empty() ? 0.0 : laser_data.maxRange);

    return payload;
}