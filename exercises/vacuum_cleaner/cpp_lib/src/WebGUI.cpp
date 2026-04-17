#include "WebGUI.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iomanip>
#include <sstream>
#include <nlohmann/json.hpp>

std::shared_ptr<WebGUI> gui = nullptr;

WebGUI::WebGUI(const std::string& host, const std::string& port, double freq)
    : MeasuringThreadingGUI(host, port, freq),
      performance_(0.0)
{
    setup_ros2();
    MeasuringThreadingGUI::start();
}

WebGUI::~WebGUI()
{
    if (executor_) {
        executor_->cancel();
    }

    if (executor_thread_.joinable()) {
        executor_thread_.join();
    }
}

void WebGUI::setup_ros2()
{
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    bridge_node_ = rclcpp::Node::make_shared("vacuum_cleaner_webgui_bridge_node");

    odom_node_ = std::make_shared<OdometryNode>("/odom", "vacuum_cleaner_odom_node");

    perf_sub_ = bridge_node_->create_subscription<gazebo_msgs::msg::PerformanceMetrics>(
        "/performance_metrics",
        10,
        std::bind(&WebGUI::performance_callback, this, std::placeholders::_1)
    );

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(bridge_node_);
    executor_->add_node(odom_node_);

    executor_thread_ = std::thread([this]() {
        executor_->spin();
    });
}

void WebGUI::performance_callback(const gazebo_msgs::msg::PerformanceMetrics::SharedPtr msg)
{
    setPerformance(msg->real_time_factor);
}

void WebGUI::setPerformance(double performance)
{
    std::lock_guard<std::mutex> lock(performance_mutex_);
    performance_ = performance;
}

double WebGUI::getCurrentPerformance()
{
    std::lock_guard<std::mutex> lock(performance_mutex_);
    return performance_;
}

std::vector<double> WebGUI::getRobotPose()
{
    Pose3d pose = odom_node_->getPose3d();

    const double x = -30.0 * pose.x + 171.0;
    const double y =  15.0 * pose.y + 63.0;
    const double yaw = pose.yaw;

    return {x, y, yaw};
}

std::vector<double> WebGUI::getPose()
{
    if (gui) {
        return gui->getRobotPose();
    }
    return {};
}

double WebGUI::getPerformance()
{
    if (gui) {
        return gui->getCurrentPerformance();
    }
    return 0.0;
}

void WebGUI::update_gui()
{
    nlohmann::json payload;

    auto pose = getRobotPose();
    if (!pose.empty()) {
        std::string pose_str = "(" +
                               std::to_string(pose[0]) + ", " +
                               std::to_string(pose[1]) + ", " +
                               std::to_string(pose[2]) + ")";
        payload["map"] = pose_str;
    }

    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << getCurrentPerformance();

    payload["brain"] = Frequency::rate;
    payload["gui"] = 20;
    payload["rtf"] = ss.str();
    payload["fps"] = -1;
    payload["lat"] = -1;

    send_to_client(payload.dump());
}