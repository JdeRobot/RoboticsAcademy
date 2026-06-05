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

// Gripper auto-attach signalling topics
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include <cmath>
#include <future>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

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

} // namespace

// Internal ROS 2 node: action clients for /Move, /Robmove and gripper, plus
// publishers for gripper attachment signalling. All blocking calls use
// promise/future so that the background executor fires result callbacks
// while the caller waits.
class HALNode : public rclcpp::Node
{
public:
    using Move    = ros2srrc_data::action::Move;
    using Robmove = ros2srrc_data::action::Robmove;
    using FJT     = control_msgs::action::FollowJointTrajectory;

    HALNode() : rclcpp::Node("hal_node")
    {
        move_client_    = rclcpp_action::create_client<Move>(this, "/Move");
        robmove_client_ = rclcpp_action::create_client<Robmove>(this, "/Robmove");
        gripper_client_ = rclcpp_action::create_client<FJT>(
            this, "/gripper_controller/follow_joint_trajectory");

        auto_attach_pub_ = this->create_publisher<std_msgs::msg::Bool>("/gripper_auto_attach", 10);
        graspable_pub_   = this->create_publisher<std_msgs::msg::String>("/graspable_objects", 10);

        RCLCPP_INFO(get_logger(), "Waiting for /Move action server...");
        while (!move_client_->wait_for_action_server(1s))
            RCLCPP_INFO(get_logger(), "Waiting for /Move...");

        RCLCPP_INFO(get_logger(), "Waiting for /Robmove action server...");
        while (!robmove_client_->wait_for_action_server(1s))
            RCLCPP_INFO(get_logger(), "Waiting for /Robmove...");

        RCLCPP_INFO(get_logger(), "Waiting for gripper action server...");
        while (!gripper_client_->wait_for_action_server(1s))
            RCLCPP_INFO(get_logger(), "Waiting for gripper controller...");

        std_msgs::msg::String graspable_msg;
        graspable_msg.data = "blue_ball,green_cylinder,yellow_box,red_box";
        graspable_pub_->publish(graspable_msg);

        // Periodic republish so late-joining subscribers receive the list
        graspable_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                std_msgs::msg::String msg;
                msg.data = "blue_ball,green_cylinder,yellow_box,red_box";
                graspable_pub_->publish(msg);
            });

        RCLCPP_INFO(get_logger(), "HAL ready");
    }

    void publish_auto_attach(bool enabled)
    {
        std_msgs::msg::Bool msg;
        msg.data = enabled;
        auto_attach_pub_->publish(msg);
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

private:
    rclcpp_action::Client<Move>::SharedPtr    move_client_;
    rclcpp_action::Client<Robmove>::SharedPtr robmove_client_;
    rclcpp_action::Client<FJT>::SharedPtr     gripper_client_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   auto_attach_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr graspable_pub_;
    rclcpp::TimerBase::SharedPtr graspable_timer_;
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
    // Enable contact-based auto-attach when closing, release when opening.
    hal_node_->publish_auto_attach(relative_closure > 5.0);

    // Convert percentage [0,100] to joint position [0.0,1.0]
    double position = relative_closure / 100.0;

    if (!hal_node_->gripper_execute(position, wait_time))
        RCLCPP_ERROR(rclcpp::get_logger("HAL"), "GripperSet failed");
}
