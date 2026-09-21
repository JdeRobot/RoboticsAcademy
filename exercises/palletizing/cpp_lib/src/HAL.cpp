#include "HAL.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// IFRA action and message types
#include "ros2srrc_data/action/move.hpp"
#include "ros2srrc_data/action/robmove.hpp"
#include "ros2srrc_data/msg/joints.hpp"
#include "ros2srrc_data/msg/joint.hpp"
#include "ros2srrc_data/msg/xyz.hpp"
#include "ros2srrc_data/msg/ypr.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <cmath>
#include <future>
#include <thread>
#include <chrono>
#include <mutex>
#include <set>
#include <map>
#include <optional>

using namespace std::chrono_literals;
using json = nlohmann::json;

namespace {

// Convert YPR in degrees to quaternion (ZYX Euler convention, same as Python HAL)
void ypr_to_quat(const std::array<double, 3>& ypr,
                 double& qx, double& qy, double& qz, double& qw)
{
    double roll  = ypr[0] * M_PI / 180.0;
    double pitch = ypr[1] * M_PI / 180.0;
    double yaw   = ypr[2] * M_PI / 180.0;

    double sr2 = std::sin(roll  / 2), cr2 = std::cos(roll  / 2);
    double sp2 = std::sin(pitch / 2), cp2 = std::cos(pitch / 2);
    double sy2 = std::sin(yaw   / 2), cy2 = std::cos(yaw   / 2);

    qx = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
    qy = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
    qz = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;
    qw = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
}

// The attacher uses substring matching, so "box" covers feeder-generated names.
const char* GRASPABLE_OBJECTS = "box";

} // namespace

// Internal ROS 2 node: action clients for /Move and /Robmove, the suction
// auto-attach publisher, and the box feeder protocol (/box_ready, /box_info,
// /pallet_info, /box_done). All blocking calls poll the state filled in by
// the background executor thread rather than spinning themselves.
class HALNode : public rclcpp::Node
{
public:
    using Move    = ros2srrc_data::action::Move;
    using Robmove = ros2srrc_data::action::Robmove;

    HALNode() : rclcpp::Node("hal_node")
    {
        move_client_    = rclcpp_action::create_client<Move>(this, "/Move");
        robmove_client_ = rclcpp_action::create_client<Robmove>(this, "/Robmove");

        auto_attach_pub_ = this->create_publisher<std_msgs::msg::Bool>("/gripper_auto_attach", 10);
        graspable_pub_   = this->create_publisher<std_msgs::msg::String>("/graspable_objects", 10);
        box_done_pub_    = this->create_publisher<std_msgs::msg::String>("/box_done", 10);

        box_ready_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/box_ready", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) { on_box_ready(msg->data); });
        box_info_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/box_info", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) { on_box_info(msg->data); });
        pallet_info_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/pallet_info", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) { on_pallet_info(msg->data); });

        RCLCPP_INFO(get_logger(), "Waiting for /Move action server...");
        while (!move_client_->wait_for_action_server(1s))
            RCLCPP_INFO(get_logger(), "Waiting for /Move...");

        RCLCPP_INFO(get_logger(), "Waiting for /Robmove action server...");
        while (!robmove_client_->wait_for_action_server(1s))
            RCLCPP_INFO(get_logger(), "Waiting for /Robmove...");

        publish_graspable();
        graspable_timer_ = this->create_wall_timer(1s, [this]() { publish_graspable(); });

        RCLCPP_INFO(get_logger(), "HAL ready");
    }

    void publish_auto_attach(bool enabled)
    {
        std_msgs::msg::Bool msg;
        msg.data = enabled;
        auto_attach_pub_->publish(msg);
    }

    // Send a /Move action goal and block until the result arrives.
    bool move_execute(const Move::Goal& goal, double wait_time)
    {
        std::promise<bool> prom;
        auto fut = prom.get_future();

        auto opts = rclcpp_action::Client<Move>::SendGoalOptions();
        opts.result_callback =
            [&prom](const rclcpp_action::ClientGoalHandle<Move>::WrappedResult& r) {
                prom.set_value(r.result->result.find("FAILED") == std::string::npos);
            };

        move_client_->async_send_goal(goal, opts);
        bool ok = fut.get();

        if (wait_time > 0.0)
            std::this_thread::sleep_for(std::chrono::duration<double>(wait_time));

        return ok;
    }

    // Send a /Robmove action goal and block until the result arrives.
    bool robmove_execute(const Robmove::Goal& goal, double wait_time)
    {
        std::promise<bool> prom;
        auto fut = prom.get_future();

        auto opts = rclcpp_action::Client<Robmove>::SendGoalOptions();
        opts.result_callback =
            [&prom](const rclcpp_action::ClientGoalHandle<Robmove>::WrappedResult& r) {
                prom.set_value(r.result->success);
            };

        robmove_client_->async_send_goal(goal, opts);
        bool ok = fut.get();

        if (wait_time > 0.0)
            std::this_thread::sleep_for(std::chrono::duration<double>(wait_time));

        return ok;
    }

    // Blocks until all joint velocities stay below threshold for stable_count cycles.
    bool wait_motion_complete(double timeout = 15.0, double vel_threshold = 0.01, int stable_count = 8)
    {
        std::mutex latest_mtx;
        sensor_msgs::msg::JointState::SharedPtr latest;

        auto sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(latest_mtx);
                latest = msg;
            });

        int stable = 0;
        auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
        bool settled = false;
        while (std::chrono::steady_clock::now() < deadline) {
            sensor_msgs::msg::JointState::SharedPtr msg;
            {
                std::lock_guard<std::mutex> lk(latest_mtx);
                msg = latest;
            }
            if (msg && !msg->velocity.empty()) {
                bool all_below = true;
                for (double v : msg->velocity) {
                    if (std::abs(v) >= vel_threshold) { all_below = false; break; }
                }
                if (all_below) {
                    if (++stable >= stable_count) { settled = true; break; }
                } else {
                    stable = 0;
                }
            }
            std::this_thread::sleep_for(50ms);
        }
        return settled;
    }

    std::string wait_for_box()
    {
        while (rclcpp::ok()) {
            {
                std::lock_guard<std::mutex> lk(mutex_);
                if (!ready_box_.empty()) {
                    std::string name = ready_box_;
                    ready_box_.clear();
                    processed_boxes_.insert(name);
                    return name;
                }
            }
            std::this_thread::sleep_for(50ms);
        }
        return "";
    }

    json get_box_info(const std::string& name, double timeout)
    {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
        while (std::chrono::steady_clock::now() < deadline) {
            {
                std::lock_guard<std::mutex> lk(mutex_);
                auto it = box_infos_.find(name);
                if (it != box_infos_.end()) return it->second;
            }
            std::this_thread::sleep_for(50ms);
        }
        throw std::runtime_error("No semantic /box_info received for " + name);
    }

    json get_pickup_pose(const std::string& name, double timeout)
    {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
        while (std::chrono::steady_clock::now() < deadline) {
            {
                std::lock_guard<std::mutex> lk(mutex_);
                auto it = pickup_poses_.find(name);
                if (it != pickup_poses_.end()) return it->second;
            }
            std::this_thread::sleep_for(50ms);
        }
        throw std::runtime_error("No pickup pose received for " + name);
    }

    json get_pallet_info(double timeout)
    {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
        while (std::chrono::steady_clock::now() < deadline) {
            {
                std::lock_guard<std::mutex> lk(mutex_);
                if (pallet_info_.has_value()) return *pallet_info_;
            }
            std::this_thread::sleep_for(50ms);
        }
        throw std::runtime_error("No /pallet_info received");
    }

    void box_done(const std::string& name)
    {
        std_msgs::msg::String msg;
        msg.data = name;
        box_done_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "BoxDone(%s) -> released next box", name.c_str());
    }

private:
    void publish_graspable()
    {
        std_msgs::msg::String msg;
        msg.data = GRASPABLE_OBJECTS;
        graspable_pub_->publish(msg);
    }

    // Ignore re-announcements of a box we've already handed to the solution.
    void on_box_ready(const std::string& name)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        if (processed_boxes_.find(name) == processed_boxes_.end())
            ready_box_ = name;
    }

    void on_box_info(const std::string& data)
    {
        try {
            json info = json::parse(data);
            std::string name = info.at("name").get<std::string>();
            json pickup_pose = info.at("pickup_pose");

            if (pickup_pose.value("frame", "") != "base_link")
                throw std::runtime_error("pickup_pose must use the base_link frame");
            if (pickup_pose.at("center").size() != 3)
                throw std::runtime_error("pickup_pose center must contain x, y, z");

            json box_info;
            box_info["name"] = name;
            box_info["sku"]  = info.at("sku");
            box_info["size"] = info.at("size");
            box_info["mass"] = info.at("mass");

            std::lock_guard<std::mutex> lk(mutex_);
            box_infos_[name] = box_info;
            pickup_poses_[name] = pickup_pose;
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Ignoring invalid /box_info message: %s", e.what());
        }
    }

    void on_pallet_info(const std::string& data)
    {
        try {
            std::lock_guard<std::mutex> lk(mutex_);
            pallet_info_ = json::parse(data);
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Ignoring invalid /pallet_info JSON: %s", e.what());
        }
    }

    rclcpp_action::Client<Move>::SharedPtr    move_client_;
    rclcpp_action::Client<Robmove>::SharedPtr robmove_client_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   auto_attach_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr graspable_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr box_done_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr box_ready_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr box_info_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pallet_info_sub_;
    rclcpp::TimerBase::SharedPtr graspable_timer_;

    std::mutex mutex_;
    std::string ready_box_;
    std::set<std::string> processed_boxes_;
    std::map<std::string, json> box_infos_;
    std::map<std::string, json> pickup_poses_;
    std::optional<json> pallet_info_;
};

// Static member definitions
std::shared_ptr<HALNode> HAL::hal_node_  = nullptr;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> HAL::executor_ = nullptr;
std::thread HAL::spin_thread_;

void HAL::init()
{
    if (hal_node_) return;
    hal_node_ = std::make_shared<HALNode>();
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(hal_node_);
    spin_thread_ = std::thread([]() { executor_->spin(); });
    spin_thread_.detach();
}

void HAL::MoveAbsJ(const std::array<double, 6>& joints, double speed, double wait_time)
{
    using Move = ros2srrc_data::action::Move;
    Move::Goal goal;
    goal.action       = "MoveJ";
    goal.speed        = static_cast<float>(speed);
    goal.movej.joint1 = joints[0];
    goal.movej.joint2 = joints[1];
    goal.movej.joint3 = joints[2];
    goal.movej.joint4 = joints[3];
    goal.movej.joint5 = joints[4];
    goal.movej.joint6 = joints[5];

    if (!hal_node_->move_execute(goal, wait_time))
        RCLCPP_ERROR(rclcpp::get_logger("HAL"), "MoveAbsJ failed");
}

void HAL::MoveSingleJ(int joint_number, double relative_angle, double speed, double wait_time)
{
    using Move = ros2srrc_data::action::Move;
    Move::Goal goal;
    goal.action       = "MoveR";
    goal.speed        = static_cast<float>(speed);
    goal.mover.joint  = std::to_string(joint_number);
    goal.mover.value  = relative_angle;

    if (!hal_node_->move_execute(goal, wait_time))
        RCLCPP_ERROR(rclcpp::get_logger("HAL"), "MoveSingleJ failed");
}

// Build a Robmove goal for LIN or PTP Cartesian moves
static ros2srrc_data::action::Robmove::Goal make_robmove_goal(
    const std::string& type,
    double speed,
    const std::array<double, 3>& xyz,
    const std::array<double, 3>& ypr)
{
    ros2srrc_data::action::Robmove::Goal goal;
    goal.type  = type;
    goal.speed = static_cast<float>(speed);
    goal.x     = xyz[0];
    goal.y     = xyz[1];
    goal.z     = xyz[2];
    ypr_to_quat(ypr, goal.qx, goal.qy, goal.qz, goal.qw);
    return goal;
}

void HAL::MoveLinear(const std::array<double, 3>& xyz, const std::array<double, 3>& ypr,
                     double speed, double wait_time)
{
    auto goal = make_robmove_goal("LIN", speed, xyz, ypr);
    if (!hal_node_->robmove_execute(goal, wait_time))
        RCLCPP_ERROR(rclcpp::get_logger("HAL"), "MoveLinear failed");
}

void HAL::MoveJoint(const std::array<double, 3>& xyz, const std::array<double, 3>& ypr,
                    double speed, double wait_time)
{
    auto goal = make_robmove_goal("PTP", speed, xyz, ypr);
    if (!hal_node_->robmove_execute(goal, wait_time))
        RCLCPP_ERROR(rclcpp::get_logger("HAL"), "MoveJoint failed");
}

void HAL::MoveRelLinear(const std::array<double, 3>& xyz, double speed, double wait_time)
{
    using Move = ros2srrc_data::action::Move;
    Move::Goal goal;
    goal.action  = "MoveL";
    goal.speed   = static_cast<float>(speed);
    goal.movel.x = xyz[0];
    goal.movel.y = xyz[1];
    goal.movel.z = xyz[2];

    if (hal_node_->move_execute(goal, wait_time))
        hal_node_->wait_motion_complete();
    else
        RCLCPP_ERROR(rclcpp::get_logger("HAL"), "MoveRelLinear failed");
}

void HAL::MoveRelReor(const std::array<double, 3>& ypr, double speed, double wait_time)
{
    using Move = ros2srrc_data::action::Move;
    Move::Goal goal;
    goal.action        = "MoveROT";
    goal.speed         = static_cast<float>(speed);
    // Python convention: ypr[0] -> pitch, ypr[1] -> yaw, ypr[2] -> roll
    goal.moverot.pitch = ypr[0];
    goal.moverot.yaw   = ypr[1];
    goal.moverot.roll  = ypr[2];

    if (!hal_node_->move_execute(goal, wait_time))
        RCLCPP_ERROR(rclcpp::get_logger("HAL"), "MoveRelReor failed");
}

void HAL::SuctionSet(bool on, double wait_time)
{
    hal_node_->publish_auto_attach(on);

    if (wait_time > 0.0)
        std::this_thread::sleep_for(std::chrono::duration<double>(wait_time));
}

std::string HAL::WaitForBox()
{
    return hal_node_->wait_for_box();
}

json HAL::GetBoxInfo(const std::string& name, double timeout)
{
    return hal_node_->get_box_info(name, timeout);
}

json HAL::GetPickupPose(const std::string& name, double timeout)
{
    return hal_node_->get_pickup_pose(name, timeout);
}

json HAL::GetPalletInfo(double timeout)
{
    return hal_node_->get_pallet_info(timeout);
}

void HAL::BoxDone(const std::string& name)
{
    hal_node_->box_done(name);
}
