#include "HAL.hpp"
#include "jderobot_drones_cpp/drone_wrapper.hpp"
#include "common_interfaces_cpp/hal/camera.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

using namespace std::chrono_literals;

std::shared_ptr<DroneWrapper> HAL::drone_node_ = nullptr;
std::shared_ptr<CameraNode> HAL::frontal_camera_node_ = nullptr;
std::shared_ptr<CameraNode> HAL::ventral_camera_node_ = nullptr;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> HAL::executor_ = nullptr;
std::thread HAL::spin_thread_;

void HAL::init()
{
    if (!drone_node_) {
        drone_node_ = std::make_shared<DroneWrapper>("drone");
        
        // Setup standard CameraNodes with their respective topics
        frontal_camera_node_ = std::make_shared<CameraNode>("/drone/frontal_cam/image_raw", "hal_frontal_camera");
        ventral_camera_node_ = std::make_shared<CameraNode>("/drone/ventral_cam/image_raw", "hal_ventral_camera");

        // MultiThreadedExecutor spins nodes concurrently
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor_->add_node(drone_node_);
        executor_->add_node(frontal_camera_node_);
        executor_->add_node(ventral_camera_node_);

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

// Motion commands

void HAL::set_cmd_pos(float x, float y, float z, float az)
{
    if (drone_node_) drone_node_->setCmdPos(x, y, z, az);
}

void HAL::set_cmd_vel(float vx, float vy, float vz, float az)
{
    if (drone_node_) drone_node_->setCmdVel(vx, vy, vz, az);
}

void HAL::set_cmd_mix(float vx, float vy, float z, float az)
{
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

// Beacon ground truth positions (world frame), from the world file.
//   initial (0.0, 0.0, 0.3)
//   beacon1 (0.0, 18.0, 0.5)
//   beacon2 (18.0, 0.0, 2.0)
//   beacon3 (0.0, -18.0, 3.5)
//   beacon4 (-18.0, 0.0, 5.0)
//   beacon5 (23.0, 23.0, 6.5)