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

// Gripper (Harmonic): FollowJointTrajectory action on /gripper_controller
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

// Link attachment services (Gazebo plugin)
#include "linkattacher_msgs/srv/attach_link.hpp"
#include "linkattacher_msgs/srv/detach_link.hpp"

#include <cmath>
#include <future>
#include <thread>
#include <chrono>
#include <unordered_map>

using namespace std::chrono_literals;

namespace {

// Map from item model name to the specific link used for attachment
const std::unordered_map<std::string, std::string> kLinkMap = {
    {"blue_ball",      "link_3"},
    {"green_cylinder", "link_2"},
    {"red_box",        "link"},
    {"yellow_box",     "link"},
};

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

} // namespace

// Internal ROS 2 node: action clients for /Move and /Robmove, gripper, and
// service clients for link attachment. All blocking calls use promise/future
// so that the background executor fires result callbacks while the caller waits.
class HALNode : public rclcpp::Node
{
public:
    using Move    = ros2srrc_data::action::Move;
    using Robmove = ros2srrc_data::action::Robmove;
    using FJT     = control_msgs::action::FollowJointTrajectory;
    using Attach  = linkattacher_msgs::srv::AttachLink;
    using Detach  = linkattacher_msgs::srv::DetachLink;

    std::string grasped_object;

    HALNode() : rclcpp::Node("hal_node")
    {
        move_client_    = rclcpp_action::create_client<Move>(this, "/Move");
        robmove_client_ = rclcpp_action::create_client<Robmove>(this, "/Robmove");
        gripper_client_ = rclcpp_action::create_client<FJT>(
            this, "/gripper_controller/follow_joint_trajectory");
        attach_client_  = this->create_client<Attach>("/ATTACHLINK");
        detach_client_  = this->create_client<Detach>("/DETACHLINK");

        RCLCPP_INFO(get_logger(), "Waiting for /Move action server...");
        while (!move_client_->wait_for_action_server(1s))
            RCLCPP_INFO(get_logger(), "Waiting for /Move...");

        RCLCPP_INFO(get_logger(), "Waiting for /Robmove action server...");
        while (!robmove_client_->wait_for_action_server(1s))
            RCLCPP_INFO(get_logger(), "Waiting for /Robmove...");

        RCLCPP_INFO(get_logger(), "Waiting for gripper action server...");
        while (!gripper_client_->wait_for_action_server(1s))
            RCLCPP_INFO(get_logger(), "Waiting for gripper controller...");

        RCLCPP_INFO(get_logger(), "Waiting for /ATTACHLINK service...");
        while (!attach_client_->wait_for_service(1s))
            RCLCPP_INFO(get_logger(), "Waiting for /ATTACHLINK...");

        RCLCPP_INFO(get_logger(), "Waiting for /DETACHLINK service...");
        while (!detach_client_->wait_for_service(1s))
            RCLCPP_INFO(get_logger(), "Waiting for /DETACHLINK...");

        RCLCPP_INFO(get_logger(), "HAL ready");
    }

    // Send a /Move action goal and block until the result arrives.
    // Returns true on success, false on failure.
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

    // Send a FollowJointTrajectory goal to the Harmonic gripper controller and block.
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

    // Call /ATTACHLINK service and block until response.
    bool attach_link(const std::string& model2, const std::string& link2)
    {
        auto req = std::make_shared<Attach::Request>();
        req->model1_name = "ur5_robotiq";
        req->link1_name  = "wrist_3_link";
        req->model2_name = model2;
        req->link2_name  = link2;

        std::promise<bool> prom;
        auto fut = prom.get_future();

        attach_client_->async_send_request(req,
            [&prom](rclcpp::Client<Attach>::SharedFuture resp) {
                prom.set_value(resp.get()->success);
            });

        return fut.get();
    }

    // Call /DETACHLINK service and block until response.
    bool detach_link(const std::string& model2, const std::string& link2)
    {
        auto req = std::make_shared<Detach::Request>();
        req->model1_name = "ur5_robotiq";
        req->link1_name  = "wrist_3_link";
        req->model2_name = model2;
        req->link2_name  = link2;

        std::promise<bool> prom;
        auto fut = prom.get_future();

        detach_client_->async_send_request(req,
            [&prom](rclcpp::Client<Detach>::SharedFuture resp) {
                prom.set_value(resp.get()->success);
            });

        return fut.get();
    }

private:
    rclcpp_action::Client<Move>::SharedPtr    move_client_;
    rclcpp_action::Client<Robmove>::SharedPtr robmove_client_;
    rclcpp_action::Client<FJT>::SharedPtr     gripper_client_;
    rclcpp::Client<Attach>::SharedPtr         attach_client_;
    rclcpp::Client<Detach>::SharedPtr         detach_client_;
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

    if (!hal_node_->move_execute(goal, wait_time))
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

void HAL::GripperSet(double relative_closure, double wait_time)
{
    // Convert percentage [0,100] to joint position [0.0,1.0]
    double position = relative_closure / 100.0;

    if (!hal_node_->gripper_execute(position, wait_time))
        RCLCPP_ERROR(rclcpp::get_logger("HAL"), "GripperSet failed");

    // Auto-detach when gripper is opened (same threshold as Python HAL_Harmonic)
    if (relative_closure <= 5.0)
        dettach();
}

void HAL::attach(const std::string& item)
{
    auto it = kLinkMap.find(item);
    if (it == kLinkMap.end()) {
        RCLCPP_ERROR(rclcpp::get_logger("HAL"), "Unknown item: %s", item.c_str());
        return;
    }

    bool ok = hal_node_->attach_link(item, it->second);
    if (ok) {
        RCLCPP_INFO(rclcpp::get_logger("HAL"), "Attached %s", item.c_str());
        hal_node_->grasped_object = item;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("HAL"), "Attach failed for %s", item.c_str());
    }
}

void HAL::dettach()
{
    if (!hal_node_ || hal_node_->grasped_object.empty()) return;

    const std::string& item = hal_node_->grasped_object;
    auto it = kLinkMap.find(item);
    if (it == kLinkMap.end()) return;

    bool ok = hal_node_->detach_link(item, it->second);
    if (ok) {
        RCLCPP_INFO(rclcpp::get_logger("HAL"), "Detached %s", item.c_str());
        hal_node_->grasped_object.clear();
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("HAL"), "Detach failed for %s", item.c_str());
    }
}
