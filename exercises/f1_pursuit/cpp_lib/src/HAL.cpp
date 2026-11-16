#include "HAL.hpp"
#include "common_interfaces_cpp/hal/motors.hpp"
#include "common_interfaces_cpp/hal/camera.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <mutex>
#include <chrono>
#include <nlohmann/json.hpp>

using namespace std::chrono_literals;

// The rival referees the race and latches how it ended
class PursuitStatusNode : public rclcpp::Node
{
public:
    PursuitStatusNode() : Node("hal_pursuit_status"), state_("racing")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        sub_ = create_subscription<std_msgs::msg::String>(
            "/f1_pursuit/status", qos,
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
std::shared_ptr<CameraNode> HAL::camera_node_ = nullptr;
std::shared_ptr<PursuitStatusNode> HAL::status_node_ = nullptr;
std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> HAL::executor_ = nullptr;
std::thread HAL::spin_thread_;

void HAL::init()
{
    if (!motors_node_) {
        motors_node_ = std::make_shared<MotorsNode>("/f1/cmd_vel", 4.0, 0.3, "hal_motors");
        camera_node_ = std::make_shared<CameraNode>("/f1/camera/image_raw", "hal_camera");
        status_node_ = std::make_shared<PursuitStatusNode>();

        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(motors_node_);
        executor_->add_node(camera_node_);
        executor_->add_node(status_node_);

        spin_thread_ = std::thread([]() {
            executor_->spin();
        });
        spin_thread_.detach();
    }
}

bool HAL::is_caught()
{
    return status_node_ && status_node_->state() == "caught";
}

// The race is over once the rival is caught or gets away, so the car stops
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