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

// Gripper
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

// Gripper auto-attach signalling topics
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

// Robot info
#include "sensor_msgs/msg/joint_state.hpp"

// TF2
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// Perception filter topics
#include "pcl_filter_msgs/msg/color_filter.hpp"
#include "pcl_filter_msgs/msg/shape_filter.hpp"

// Camera
#include "common_interfaces_cpp/hal/camera.hpp"

// YAML and package path
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cmath>
#include <fstream>
#include <future>
#include <thread>
#include <chrono>
#include <mutex>
#include <unordered_map>
#include <stdexcept>

using namespace std::chrono_literals;

namespace {

// Maps color/shape names to the integer codes expected by the PCL filter service.
int color_to_int(const std::string& color)
{
    if (color == "red")    return 1;
    if (color == "green")  return 2;
    if (color == "blue")   return 3;
    if (color == "purple") return 4;
    return 0;
}

int shape_to_int(const std::string& shape)
{
    if (shape == "sphere")   return 1;
    if (shape == "cylinder") return 2;
    return 0;
}

// Convert YPR in degrees to quaternion (ZYX Euler convention).
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

} // namespace

// Storage for one workspace object loaded from YAML.
struct ObjectData {
    std::array<double, 3> position;
    double height, width, length;
    std::string shape, color;
};

// Internal ROS 2 node: action clients, publishers/subscribers for the full
// machine-vision HAL. All blocking calls use promise/future so that the
// background executor fires callbacks while the caller waits.
class HALNode : public rclcpp::Node
{
public:
    using Move    = ros2srrc_data::action::Move;
    using Robmove = ros2srrc_data::action::Robmove;
    using FJT     = control_msgs::action::FollowJointTrajectory;

    // Shared robot state (written by subscriber callbacks, read by HAL API).
    std::array<double, 6> joint_states{};
    bool                  joint_states_ready{false};
    mutable std::mutex    joint_mutex;

    // Workspace objects and targets loaded from YAML at construction.
    std::unordered_map<std::string, ObjectData>       object_map;
    std::unordered_map<std::string, std::array<double,3>> target_map;

    HALNode() : rclcpp::Node("hal_node")
    {
        move_client_    = rclcpp_action::create_client<Move>(this, "/Move");
        robmove_client_ = rclcpp_action::create_client<Robmove>(this, "/Robmove");
        gripper_client_ = rclcpp_action::create_client<FJT>(
            this, "/gripper_controller/follow_joint_trajectory");

        auto_attach_pub_ = this->create_publisher<std_msgs::msg::Bool>("/gripper_auto_attach", 10);
        graspable_pub_   = this->create_publisher<std_msgs::msg::String>("/graspable_objects", 10);
        env_scan_pub_    = this->create_publisher<std_msgs::msg::Bool>("/trigger_env_scan", 10);
        color_filter_pub_ = this->create_publisher<pcl_filter_msgs::msg::ColorFilter>(
            "/start_color_filter", 10);
        shape_filter_pub_ = this->create_publisher<pcl_filter_msgs::msg::ShapeFilter>(
            "/start_shape_filter", 10);

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(joint_mutex);
                const std::vector<std::string> order = {
                    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
                };
                for (std::size_t i = 0; i < order.size(); ++i) {
                    auto it = std::find(msg->name.begin(), msg->name.end(), order[i]);
                    if (it != msg->name.end())
                        joint_states[i] = msg->position[it - msg->name.begin()] * 180.0 / M_PI;
                }
                joint_states_ready = true;
            });

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(get_logger(), "Waiting for /Move action server...");
        while (!move_client_->wait_for_action_server(1s))
            RCLCPP_INFO(get_logger(), "Waiting for /Move...");

        RCLCPP_INFO(get_logger(), "Waiting for /Robmove action server...");
        while (!robmove_client_->wait_for_action_server(1s))
            RCLCPP_INFO(get_logger(), "Waiting for /Robmove...");

        RCLCPP_INFO(get_logger(), "Waiting for gripper action server...");
        while (!gripper_client_->wait_for_action_server(1s))
            RCLCPP_INFO(get_logger(), "Waiting for gripper controller...");

        // Publish graspable objects list immediately and keep republishing.
        std_msgs::msg::String graspable_msg;
        graspable_msg.data = "blue_sphere,red_sphere,green_sphere,purple_sphere,"
                             "green_cylinder,purple_cylinder,red_cylinder";
        graspable_pub_->publish(graspable_msg);

        graspable_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                std_msgs::msg::String msg;
                msg.data = "blue_sphere,red_sphere,green_sphere,purple_sphere,"
                           "green_cylinder,purple_cylinder,red_cylinder";
                graspable_pub_->publish(msg);
            });

        load_workspace();
        RCLCPP_INFO(get_logger(), "HAL ready");
    }

    // ---- Publish helpers ----

    void publish_auto_attach(bool enabled)
    {
        std_msgs::msg::Bool msg;
        msg.data = enabled;
        auto_attach_pub_->publish(msg);
    }

    void trigger_env_scan(bool start)
    {
        std_msgs::msg::Bool msg;
        msg.data = start;
        env_scan_pub_->publish(msg);
    }

    void pub_color_filter(const std::string& color,
                          int rmax, int rmin,
                          int gmax, int gmin,
                          int bmax, int bmin,
                          bool status)
    {
        pcl_filter_msgs::msg::ColorFilter msg;
        msg.color  = color_to_int(color);
        msg.rmax = rmax; msg.rmin = rmin;
        msg.gmax = gmax; msg.gmin = gmin;
        msg.bmax = bmax; msg.bmin = bmin;
        msg.status = status;
        color_filter_pub_->publish(msg);
    }

    void pub_shape_filter(const std::string& color, const std::string& shape,
                          double radius, bool status)
    {
        pcl_filter_msgs::msg::ShapeFilter msg;
        msg.color  = color_to_int(color);
        msg.shape  = shape_to_int(shape);
        msg.radius = static_cast<float>(radius);
        msg.status = status;
        shape_filter_pub_->publish(msg);
    }

    // ---- TCP pose via TF2 ----

    std::array<double, 3> get_tcp_position()
    {
        auto t = tf_buffer_->lookupTransform("world", "tool0", tf2::TimePointZero, 1s);
        return { t.transform.translation.x,
                 t.transform.translation.y,
                 t.transform.translation.z };
    }

    std::array<double, 3> get_tcp_orientation()
    {
        auto t = tf_buffer_->lookupTransform("world", "tool0", tf2::TimePointZero, 1s);

        double qx = t.transform.rotation.x;
        double qy = t.transform.rotation.y;
        double qz = t.transform.rotation.z;
        double qw = t.transform.rotation.w;

        double sinr = 2.0 * (qw * qx + qy * qz);
        double cosr = 1.0 - 2.0 * (qx * qx + qy * qy);
        double roll  = std::atan2(sinr, cosr);

        double sinp  = 2.0 * (qw * qy - qz * qx);
        double pitch = std::abs(sinp) >= 1.0
                           ? std::copysign(M_PI / 2.0, sinp)
                           : std::asin(sinp);

        double siny = 2.0 * (qw * qz + qx * qy);
        double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
        double yaw   = std::atan2(siny, cosy);

        return { yaw   * 180.0 / M_PI,
                 pitch * 180.0 / M_PI,
                 roll  * 180.0 / M_PI };
    }

    // ---- Robot action execution ----

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

    bool gripper_execute(double position, double wait_time)
    {
        FJT::Goal goal;
        goal.trajectory.joint_names = {"robotiq_85_left_knuckle_joint"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {position};
        point.time_from_start.sec = 1;
        goal.trajectory.points.push_back(point);

        std::promise<bool> prom;
        auto fut = prom.get_future();

        auto opts = rclcpp_action::Client<FJT>::SendGoalOptions();
        opts.result_callback =
            [&prom](const rclcpp_action::ClientGoalHandle<FJT>::WrappedResult& r) {
                prom.set_value(r.code == rclcpp_action::ResultCode::SUCCEEDED);
            };

        gripper_client_->async_send_goal(goal, opts);
        bool ok = fut.get();

        if (wait_time > 0.0)
            std::this_thread::sleep_for(std::chrono::duration<double>(wait_time));

        return ok;
    }

private:
    // ---- ROS 2 interfaces ----
    rclcpp_action::Client<Move>::SharedPtr    move_client_;
    rclcpp_action::Client<Robmove>::SharedPtr robmove_client_;
    rclcpp_action::Client<FJT>::SharedPtr     gripper_client_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   auto_attach_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr graspable_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   env_scan_pub_;
    rclcpp::Publisher<pcl_filter_msgs::msg::ColorFilter>::SharedPtr color_filter_pub_;
    rclcpp::Publisher<pcl_filter_msgs::msg::ShapeFilter>::SharedPtr shape_filter_pub_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr graspable_timer_;

    // ---- YAML workspace loader ----

    void load_workspace()
    {
        // Mirror the Python HAL: prefer the mounted source path, fall back to installed package.
        const std::string hardcoded =
            "/resources/exercises/machine_vision/models_info.yaml";

        std::string path;
        {
            std::ifstream f(hardcoded);
            if (f.good()) path = hardcoded;
        }

        if (path.empty()) {
            try {
                path = "/resources/exercises/machine_vision/models_info.yaml";
            } catch (...) {
                RCLCPP_WARN(get_logger(), "models_info.yaml not found; workspace empty");
                return;
            }
        }

        load_objects(path);
        load_targets(path);
    }

    void load_objects(const std::string& path)
    {
        try {
            YAML::Node doc = YAML::LoadFile(path);
            double rx = doc["robot"]["pose"]["x"].as<double>();
            double ry = doc["robot"]["pose"]["y"].as<double>();
            double rz = doc["robot"]["pose"]["z"].as<double>();

            for (const auto& kv : doc["objects"]) {
                const std::string name  = kv.first.as<std::string>();
                const YAML::Node& spec  = kv.second;
                const std::string shape = spec["shape"].as<std::string>();
                const std::string color = spec["color"].as<std::string>();

                double x = spec["pose"]["x"].as<double>() - rx;
                double y = spec["pose"]["y"].as<double>() - ry;
                double z = spec["pose"]["z"].as<double>() - rz;

                ObjectData obj;
                obj.shape = shape;
                obj.color = color;

                if (shape == "sphere") {
                    double r = spec["size"].as<double>();
                    z += r;
                    obj.height = obj.width = obj.length = r * 2.0;
                } else if (shape == "cylinder") {
                    double h = spec["size"]["height"].as<double>();
                    double r = spec["size"]["radius"].as<double>();
                    z += h / 2.0;
                    obj.height = h;
                    obj.width  = obj.length = r * 2.0;
                } else {  // box
                    double sx = spec["size"]["x"].as<double>();
                    double sy = spec["size"]["y"].as<double>();
                    double sz = spec["size"]["z"].as<double>();
                    z += sz / 2.0;
                    obj.height = sz; obj.width = sy; obj.length = sx;
                }

                obj.position = {x, y, z};
                object_map[name] = obj;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to load objects: %s", e.what());
        }
    }

    void load_targets(const std::string& path)
    {
        try {
            YAML::Node doc = YAML::LoadFile(path);
            double rx = doc["robot"]["pose"]["x"].as<double>();
            double ry = doc["robot"]["pose"]["y"].as<double>();

            for (const auto& kv : doc["targets"]) {
                const std::string name = kv.first.as<std::string>();
                double x = kv.second["x"].as<double>() - rx;
                double y = kv.second["y"].as<double>() - ry;
                double z = kv.second["z"].as<double>();
                target_map[name] = {x, y, z};
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to load targets: %s", e.what());
        }
    }
};

// ---- Static member definitions ----
std::shared_ptr<HALNode>   HAL::hal_node_     = nullptr;
std::shared_ptr<CameraNode> HAL::hand_camera_ = nullptr;
std::shared_ptr<CameraNode> HAL::base_camera_ = nullptr;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> HAL::executor_ = nullptr;
std::thread HAL::spin_thread_;

void HAL::init()
{
    if (hal_node_) return;

    hal_node_    = std::make_shared<HALNode>();
    hand_camera_ = std::make_shared<CameraNode>("/hand_camera/image", "hal_hand_camera");
    base_camera_ = std::make_shared<CameraNode>("/base_camera/image", "hal_base_camera");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(hal_node_);
    executor_->add_node(hand_camera_);
    executor_->add_node(base_camera_);

    spin_thread_ = std::thread([]() { executor_->spin(); });
    spin_thread_.detach();
}

// ---- Build a Robmove goal ----
static ros2srrc_data::action::Robmove::Goal make_robmove_goal(
    const std::string& type, double speed,
    const std::array<double, 3>& xyz, const std::array<double, 3>& ypr)
{
    ros2srrc_data::action::Robmove::Goal goal;
    goal.type  = type;
    goal.speed = static_cast<float>(speed);
    goal.x = xyz[0]; goal.y = xyz[1]; goal.z = xyz[2];
    ypr_to_quat(ypr, goal.qx, goal.qy, goal.qz, goal.qw);
    return goal;
}

// ---- Kinematics ----

void HAL::MoveAbsJ(const std::array<double, 6>& joints, double speed, double wait_time)
{
    using Move = ros2srrc_data::action::Move;
    Move::Goal goal;
    goal.action       = "MoveJ";
    goal.speed        = static_cast<float>(speed);
    goal.movej.joint1 = joints[0]; goal.movej.joint2 = joints[1];
    goal.movej.joint3 = joints[2]; goal.movej.joint4 = joints[3];
    goal.movej.joint5 = joints[4]; goal.movej.joint6 = joints[5];

    if (!hal_node_->move_execute(goal, wait_time))
        RCLCPP_ERROR(rclcpp::get_logger("HAL"), "MoveAbsJ failed");
}

void HAL::MoveSingleJ(int joint_number, double relative_angle, double speed, double wait_time)
{
    using Move = ros2srrc_data::action::Move;
    Move::Goal goal;
    goal.action      = "MoveR";
    goal.speed       = static_cast<float>(speed);
    goal.mover.joint = std::to_string(joint_number);
    goal.mover.value = relative_angle;

    if (!hal_node_->move_execute(goal, wait_time))
        RCLCPP_ERROR(rclcpp::get_logger("HAL"), "MoveSingleJ failed");
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
    goal.movel.x = xyz[0]; goal.movel.y = xyz[1]; goal.movel.z = xyz[2];

    if (!hal_node_->move_execute(goal, wait_time))
        RCLCPP_ERROR(rclcpp::get_logger("HAL"), "MoveRelLinear failed");
}

void HAL::MoveRelReor(const std::array<double, 3>& ypr, double speed, double wait_time)
{
    using Move = ros2srrc_data::action::Move;
    Move::Goal goal;
    goal.action        = "MoveROT";
    goal.speed         = static_cast<float>(speed);
    goal.moverot.pitch = ypr[0];
    goal.moverot.yaw   = ypr[1];
    goal.moverot.roll  = ypr[2];

    if (!hal_node_->move_execute(goal, wait_time))
        RCLCPP_ERROR(rclcpp::get_logger("HAL"), "MoveRelReor failed");
}

// ---- Gripper ----

void HAL::GripperSet(double relative_closure, double wait_time)
{
    // Enable contact-based auto-attach when closing, release when opening.
    hal_node_->publish_auto_attach(relative_closure > 5.0);

    if (!hal_node_->gripper_execute(relative_closure / 100.0, wait_time))
        RCLCPP_ERROR(rclcpp::get_logger("HAL"), "GripperSet failed");
}

// ---- Robot info ----

std::array<double, 3> HAL::get_TCP_position()
{
    try {
        return hal_node_->get_tcp_position();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("HAL"), "get_TCP_position: %s", e.what());
        return {};
    }
}

std::array<double, 3> HAL::get_TCP_orientation()
{
    try {
        return hal_node_->get_tcp_orientation();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("HAL"), "get_TCP_orientation: %s", e.what());
        return {};
    }
}

std::array<double, 6> HAL::get_Joint_states()
{
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!hal_node_->joint_states_ready &&
           std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(10ms);

    std::lock_guard<std::mutex> lk(hal_node_->joint_mutex);
    return hal_node_->joint_states;
}

// ---- Cameras ----

cv::Mat HAL::getImage(const std::string& camera)
{
    CameraNode* node = (camera == "base") ? base_camera_.get() : hand_camera_.get();
    if (!node) return cv::Mat();

    auto img = node->getImage();
    while (!img && rclcpp::ok()) {
        std::this_thread::sleep_for(5ms);
        img = node->getImage();
    }
    return img ? img->data.clone() : cv::Mat();
}

// ---- Perception queries ----

std::array<double, 3> HAL::get_object_position(const std::string& object_name)
{
    auto it = hal_node_->object_map.find(object_name);
    if (it == hal_node_->object_map.end()) {
        RCLCPP_WARN(rclcpp::get_logger("HAL"), "Object '%s' not found", object_name.c_str());
        return {};
    }
    return it->second.position;
}

ObjectInfo HAL::get_object_info(const std::string& object_name)
{
    auto it = hal_node_->object_map.find(object_name);
    if (it == hal_node_->object_map.end()) {
        RCLCPP_WARN(rclcpp::get_logger("HAL"), "Object '%s' not found", object_name.c_str());
        return {};
    }
    const ObjectData& d = it->second;
    ObjectInfo info;
    info.position = d.position;
    info.height   = d.height;
    info.width    = d.width;
    info.length   = d.length;
    info.shape    = d.shape;
    info.color    = d.color;
    return info;
}

std::array<double, 3> HAL::get_target_position(const std::string& target_name)
{
    auto it = hal_node_->target_map.find(target_name);
    if (it == hal_node_->target_map.end()) {
        RCLCPP_WARN(rclcpp::get_logger("HAL"), "Target '%s' not found", target_name.c_str());
        return {};
    }
    return it->second;
}

// ---- Workspace scan ----

static const std::array<double, 6> kHome      = {0.0, -90.0, 0.0, 0.0, -90.0, 0.0};
static const std::array<double, 6> kScanPose  = {180.0, -90.0, 0.0, 0.0, -60.0, 0.0};

void HAL::scan_workspace()
{
    HAL::MoveAbsJ(kHome, 0.9, 0.5);
    HAL::GripperSet(0.0, 0.5);

    hal_node_->trigger_env_scan(true);
    HAL::MoveAbsJ(kScanPose, 0.1, 2.0);
    hal_node_->trigger_env_scan(false);

    HAL::MoveAbsJ(kHome, 0.9, 0.5);
}

void HAL::buildmap()
{
    HAL::MoveAbsJ(kHome, 0.9, 0.5);
    HAL::GripperSet(0.0, 0.5);

    hal_node_->trigger_env_scan(true);
    std::this_thread::sleep_for(1s);
    HAL::MoveAbsJ(kScanPose, 0.1, 2.0);
    hal_node_->trigger_env_scan(false);

    HAL::MoveAbsJ(kHome, 0.9, 0.5);
}

// ---- Perception filters ----

void HAL::start_color_filter(const std::string& color,
                              int rmax, int rmin,
                              int gmax, int gmin,
                              int bmax, int bmin)
{
    hal_node_->pub_color_filter(color, rmax, rmin, gmax, gmin, bmax, bmin, true);
}

void HAL::stop_color_filter(const std::string& color)
{
    hal_node_->pub_color_filter(color, 0, 0, 0, 0, 0, 0, false);
}

void HAL::start_shape_filter(const std::string& color, const std::string& shape, double radius)
{
    hal_node_->pub_shape_filter(color, shape, radius, true);
}

void HAL::stop_shape_filter(const std::string& color, const std::string& shape)
{
    hal_node_->pub_shape_filter(color, shape, 0.0, false);
}

// ---- Utility ----

double HAL::gripper_percentage_for(double diameter, double max_open_m)
{
    double pct = (1.0 - (diameter / max_open_m)) * 100.0;
    return std::max(0.0, std::min(100.0, pct));
}
