#include "HAL.hpp"
#include "common_interfaces_cpp/hal/motors.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"
#include "common_interfaces_cpp/hal/laser.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <mutex>
#include <chrono>
#include <nlohmann/json.hpp>

using namespace std::chrono_literals;

// The rival referees the race and latches how it ended
class RaceStatusNode : public rclcpp::Node
{
public:
    RaceStatusNode() : Node("hal_race_status"), state_("racing")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        sub_ = create_subscription<std_msgs::msg::String>(
            "/f1_race/status", qos,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                try {
                    auto status = nlohmann::json::parse(msg->data);
                    std::lock_guard<std::mutex> lk(mtx_);
                    state_ = status.at("state").get<std::string>();
                } catch (...) {
                }
            });
    }

    std::string state()
    {
        std::lock_guard<std::mutex> lk(mtx_);
        return state_;
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    std::mutex mtx_;
    std::string state_;
};

std::shared_ptr<MotorsNode> HAL::motors_node_ = nullptr;
std::shared_ptr<OdometryNode> HAL::odometry_node_ = nullptr;
std::shared_ptr<LaserNode> HAL::laser_node_ = nullptr;
std::shared_ptr<RaceStatusNode> HAL::status_node_ = nullptr;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> HAL::executor_ = nullptr;
std::thread HAL::spin_thread_;

void HAL::init()
{
    if (!motors_node_) {
        odometry_node_ = std::make_shared<OdometryNode>("/f1/odom", "hal_odom");
        motors_node_ = std::make_shared<MotorsNode>("/f1/cmd_vel", 4.0, 0.3, "hal_motors");
        laser_node_ = std::make_shared<LaserNode>("/f1/laser/scan", "hal_laser");
        status_node_ = std::make_shared<RaceStatusNode>();

        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor_->add_node(odometry_node_);
        executor_->add_node(motors_node_);
        executor_->add_node(laser_node_);
        executor_->add_node(status_node_);

        spin_thread_ = std::thread([]() {
            executor_->spin();
        });
        spin_thread_.detach();
    }
}

bool HAL::has_won()
{
    return status_node_ && status_node_->state() == "won";
}

// The race is over once either car finishes the lap, so the car stops
bool HAL::race_over()
{
    if (!status_node_ || status_node_->state() == "racing") return false;
    motors_node_->sendV(0.0);
    motors_node_->sendW(0.0);
    return true;
}

void HAL::set_v(const float velocity)
{
    if (!motors_node_ || race_over()) return;
    motors_node_->sendV(static_cast<double>(velocity));
}

void HAL::set_w(const float velocity)
{
    if (!motors_node_ || race_over()) return;
    motors_node_->sendW(static_cast<double>(velocity));
}

HAL::Pose3d HAL::get_pose3d()
{
    if (!odometry_node_) return HAL::Pose3d{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    auto raw_pose = odometry_node_->getPose3d();
    return HAL::Pose3d{
        raw_pose.x, 
        raw_pose.y, 
        raw_pose.z, 
        raw_pose.h,
        raw_pose.yaw, 
        raw_pose.pitch, 
        raw_pose.roll,
        raw_pose.timeStamp
    };
}

HAL::LaserData HAL::get_laser_data()
{
    if (!laser_node_) return HAL::LaserData{};
    
    auto raw_laser = laser_node_->getLaserData();
    while (raw_laser.values.empty() && rclcpp::ok()) {
        std::this_thread::sleep_for(5ms);
        raw_laser = laser_node_->getLaserData();
    }
    
    return HAL::LaserData{
        raw_laser.values, 
        raw_laser.minAngle, 
        raw_laser.maxAngle, 
        raw_laser.minRange, 
        raw_laser.maxRange,
        raw_laser.timeStamp
    };
}