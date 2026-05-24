#ifndef MAP_HPP_
#define MAP_HPP_

#include "common_interfaces_cpp/hal/odometry.hpp"
#include "common_interfaces_cpp/hal/laser.hpp"
#include <memory>
#include <vector>
#include <string>
#include <mutex>
#include <array>
#include <thread>
#include <atomic>
#include <nlohmann/json.hpp>

class Target {
public:
    Target(std::string id, Pose3d pose, bool active = false, bool reached = false);

    std::string get_id() const;
    Pose3d get_pose() const;
    bool is_reached() const;
    void set_reached(bool value);
    bool is_active() const;
    void set_active(bool value);

private:
    std::string id_;
    Pose3d pose_;
    bool active_;
    bool reached_;
};

class Map {
public:
    Map(std::shared_ptr<LaserNode> laser, std::shared_ptr<OdometryNode> odom);
    ~Map();

    void set_car(double newx, double newy);
    void set_obs(double newx, double newy);
    void set_avg(double newx, double newy);
    void set_target_pos(double newx, double newy);
    void set_target_x(double x);
    void set_target_y(double y);

    std::array<double, 2> get_next_target();
    void mark_current_target_reached();
    void reset();

    nlohmann::json get_json_data();

private:
    void load_targets();
    void laser_poll_loop();

    double carx_, cary_;
    double obsx_, obsy_;
    double avgx_, avgy_;
    double targetx_, targety_;

    std::vector<std::shared_ptr<Target>> targets_;
    std::shared_ptr<LaserNode> laser_;
    std::shared_ptr<OdometryNode> odom_;

    LaserData cached_laser_;
    std::mutex laser_mtx_;
    std::thread laser_thread_;
    std::atomic<bool> laser_running_;

    std::mutex map_mtx_;
};

#endif