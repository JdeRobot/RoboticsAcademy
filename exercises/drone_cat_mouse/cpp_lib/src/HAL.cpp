#include "HAL.hpp"
#include "jderobot_drones_cpp/drone_wrapper.hpp"
#include "common_interfaces_cpp/hal/camera.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <atomic>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

namespace {
constexpr const char *MOUSE_POSE_TOPIC = "/drone_mouse/self_localization/pose";
constexpr double CATCH_RADIUS = 1.8;

bool is_known(const std::vector<double> &p)
{
    return std::abs(p[0]) > 1e-6 || std::abs(p[1]) > 1e-6 || std::abs(p[2]) > 1e-6;
}
} // namespace

// Live pose of the mouse drone, published by its own self-localization.
// A ROS2-direct student subscribes to this same topic directly, without HAL.
class MousePoseNode : public rclcpp::Node
{
public:
    MousePoseNode() : rclcpp::Node("mouse_pose_hal"), x_(0.0), y_(0.0), z_(0.0)
    {
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            MOUSE_POSE_TOPIC, rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                x_.store(msg->pose.position.x);
                y_.store(msg->pose.position.y);
                z_.store(msg->pose.position.z);
            });
    }

    std::vector<double> get_position() const
    {
        return {x_.load(), y_.load(), z_.load()};
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    std::atomic<double> x_, y_, z_;
};

std::shared_ptr<DroneWrapper> HAL::drone_node_ = nullptr;
std::shared_ptr<CameraNode> HAL::frontal_camera_node_ = nullptr;
std::shared_ptr<CameraNode> HAL::ventral_camera_node_ = nullptr;
std::shared_ptr<MousePoseNode> HAL::mouse_pose_node_ = nullptr;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> HAL::executor_ = nullptr;
std::thread HAL::spin_thread_;
bool HAL::mouse_flew_ = false;
bool HAL::stopped_ = false;

void HAL::init()
{
    if (!drone_node_) {
        drone_node_ = std::make_shared<DroneWrapper>("drone");

        // Setup standard CameraNodes with their respective topics
        frontal_camera_node_ = std::make_shared<CameraNode>("/drone/frontal_cam/image_raw", "hal_frontal_camera");
        ventral_camera_node_ = std::make_shared<CameraNode>("/drone/ventral_cam/image_raw", "hal_ventral_camera");
        mouse_pose_node_ = std::make_shared<MousePoseNode>();

        // MultiThreadedExecutor spins nodes concurrently
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor_->add_node(drone_node_);
        executor_->add_node(frontal_camera_node_);
        executor_->add_node(ventral_camera_node_);
        executor_->add_node(mouse_pose_node_);

        spin_thread_ = std::thread([]() {
            executor_->spin();
        });
        spin_thread_.detach();
    }
}

cv::Mat HAL::get_frontal_image()
{
    if (!frontal_camera_node_) return cv::Mat();

    auto image = frontal_camera_node_->getImage();
    return (image) ? image->data.clone() : cv::Mat();
}

cv::Mat HAL::get_ventral_image()
{
    if (!ventral_camera_node_) return cv::Mat();

    auto image = ventral_camera_node_->getImage();
    return (image) ? image->data.clone() : cv::Mat();
}

HAL::Pose3d HAL::get_pose3d()
{
    // Safe default when node is not ready
    if (!drone_node_) return HAL::Pose3d{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    auto pos = drone_node_->getPosition();
    auto ori = drone_node_->getOrientation();

    return HAL::Pose3d{
        pos.size() > 0 ? pos[0] : 0.0,
        pos.size() > 1 ? pos[1] : 0.0,
        pos.size() > 2 ? pos[2] : 0.0,
        0.0,
        ori.size() > 2 ? ori[2] : 0.0,
        ori.size() > 1 ? ori[1] : 0.0,
        ori.size() > 0 ? ori[0] : 0.0,
        0.0
    };
}

HAL::Velocity3d HAL::get_velocity()
{
    // Safe default when node is not ready
    if (!drone_node_) return HAL::Velocity3d{0.0, 0.0, 0.0, 0.0};

    auto vel = drone_node_->getVelocity();
    float yaw_rate = drone_node_->getYawRate();

    return HAL::Velocity3d{
        vel.size() > 0 ? vel[0] : 0.0,
        vel.size() > 1 ? vel[1] : 0.0,
        vel.size() > 2 ? vel[2] : 0.0,
        static_cast<double>(yaw_rate)
    };
}

int HAL::get_landed_state()
{
    return drone_node_ ? drone_node_->getLandedState() : -1;
}

// Mouse tracking

std::vector<double> HAL::get_mouse_position()
{
    return mouse_pose_node_ ? mouse_pose_node_->get_position() : std::vector<double>{0.0, 0.0, 0.0};
}

bool HAL::is_caught()
{
    if (!mouse_pose_node_ || !drone_node_) return false;

    auto mouse = mouse_pose_node_->get_position();
    if (!is_known(mouse)) return false;

    auto here = drone_node_->getPosition();
    if (here.size() < 3 || here[2] < 1.0) return false;

    double gap = std::sqrt(
        std::pow(here[0] - mouse[0], 2) +
        std::pow(here[1] - mouse[1], 2) +
        std::pow(here[2] - mouse[2], 2));
    return gap < CATCH_RADIUS;
}

bool HAL::handle_catch()
{
    if (stopped_) return true;
    if (!mouse_pose_node_) return false;

    auto mouse = mouse_pose_node_->get_position();
    if (!is_known(mouse)) return false;

    if (mouse[2] > 1.0) mouse_flew_ = true;

    bool run_over = is_caught() || (mouse_flew_ && mouse[2] < 0.5);
    if (!run_over) return false;

    stopped_ = true;
    if (drone_node_) {
        drone_node_->setCmdVel(0.0f, 0.0f, 0.0f, 0.0f);
        drone_node_->land();
    }
    return true;
}

// Motion commands

void HAL::set_cmd_pos(float x, float y, float z, float az)
{
    if (handle_catch()) return;
    if (drone_node_) drone_node_->setCmdPos(x, y, z, az);
}

void HAL::set_cmd_vel(float vx, float vy, float vz, float az)
{
    if (handle_catch()) return;
    if (drone_node_) drone_node_->setCmdVel(vx, vy, vz, az);
}

void HAL::set_cmd_mix(float vx, float vy, float z, float az)
{
    if (handle_catch()) return;
    if (drone_node_) drone_node_->setCmdMix(vx, vy, z, az);
}

// High-level blocking commands

void HAL::takeoff(float h)
{
    if (drone_node_) drone_node_->takeoff(h);
}

void HAL::land()
{
    if (drone_node_) drone_node_->land();
}
