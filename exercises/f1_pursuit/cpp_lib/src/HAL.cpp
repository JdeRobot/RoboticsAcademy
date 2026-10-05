#include "HAL.hpp"
#include "common_interfaces_cpp/hal/motors.hpp"
#include "common_interfaces_cpp/hal/camera.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <atomic>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

// The rival publishes this once it has pulled clear of the grid
class ArmedNode : public rclcpp::Node {
public:
    ArmedNode() : rclcpp::Node("hal_armed_node"), armed_(false) {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        sub_ = create_subscription<std_msgs::msg::Bool>(
            "/f1_pursuit/armed", qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                armed_.store(msg->data);
            });
    }
    bool armed() const { return armed_.load(); }

private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
    std::atomic<bool> armed_;
};

namespace {
// Keep in step with rival.py and the Python HAL
constexpr double kCatchRadius = 2.5;

HAL::Pose3d convert(const ::Pose3d &p)
{
    return HAL::Pose3d{p.x, p.y, p.z, p.h, p.yaw, p.pitch, p.roll, p.timeStamp};
}
} 

std::shared_ptr<MotorsNode> HAL::motors_node_ = nullptr;
std::shared_ptr<CameraNode> HAL::camera_node_ = nullptr;
std::shared_ptr<OdometryNode> HAL::odom_node_ = nullptr;
std::shared_ptr<OdometryNode> HAL::rival_odom_node_ = nullptr;
std::shared_ptr<ArmedNode> HAL::armed_node_ = nullptr;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> HAL::executor_ = nullptr;
std::thread HAL::spin_thread_;
bool HAL::stopped_ = false;

void HAL::init()
{
    if (!motors_node_) {
        motors_node_ = std::make_shared<MotorsNode>("/f1/cmd_vel", 4.0, 0.3, "hal_motors");
        camera_node_ = std::make_shared<CameraNode>("/f1/camera/image_raw", "hal_camera");
        odom_node_ = std::make_shared<OdometryNode>("/f1/odom", "hal_odom");
        rival_odom_node_ = std::make_shared<OdometryNode>("/f1_rival/odom", "hal_rival_odom");
        armed_node_ = std::make_shared<ArmedNode>();

        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor_->add_node(motors_node_);
        executor_->add_node(camera_node_);
        executor_->add_node(odom_node_);
        executor_->add_node(rival_odom_node_);
        executor_->add_node(armed_node_);

        spin_thread_ = std::thread([]() {
            executor_->spin();
        });
        spin_thread_.detach();
    }
}

cv::Mat HAL::get_image()
{
    if (!camera_node_) return cv::Mat();
    auto image = camera_node_->getImage();
    while (!image && rclcpp::ok()) {
        std::this_thread::sleep_for(5ms);
        image = camera_node_->getImage();
    }
    return (image) ? image->data.clone() : cv::Mat();
}

HAL::Pose3d HAL::get_pose3d()
{
    if (!odom_node_) return HAL::Pose3d{};
    return convert(odom_node_->getPose3d());
}

HAL::Pose3d HAL::get_rival_pose3d()
{
    if (!rival_odom_node_) return HAL::Pose3d{};
    return convert(rival_odom_node_->getPose3d());
}

std::vector<double> HAL::get_rival_position()
{
    const auto pose = get_rival_pose3d();
    return {pose.x, pose.y, pose.z};
}

double HAL::get_gap()
{
    const auto here = get_pose3d();
    const auto there = get_rival_pose3d();
    return std::hypot(here.x - there.x, here.y - there.y);
}

bool HAL::is_racing()
{
    return armed_node_ && armed_node_->armed();
}

bool HAL::is_caught()
{
    if (!is_racing()) return false;
    const auto rival = get_rival_pose3d();
    const bool seen = std::fabs(rival.x) > 1e-6 ||
                      std::fabs(rival.y) > 1e-6 ||
                      std::fabs(rival.z) > 1e-6;
    if (!seen) return false;
    return get_gap() < kCatchRadius;
}

bool HAL::handle_catch()
{
    if (!is_caught()) return false;
    if (!stopped_) {
        stopped_ = true;
        if (motors_node_) {
            motors_node_->sendV(0.0);
            motors_node_->sendW(0.0);
        }
    }
    return true;
}

void HAL::set_v(const float velocity)
{
    if (handle_catch()) return;
    if (motors_node_) motors_node_->sendV(static_cast<double>(velocity));
}

void HAL::set_w(const float velocity)
{
    if (handle_catch()) return;
    if (motors_node_) motors_node_->sendW(static_cast<double>(velocity));
}
