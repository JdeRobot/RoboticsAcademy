#ifndef WEBGUI_HPP_
#define WEBGUI_HPP_

#include <memory>
#include <thread>
#include <mutex>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/msg/performance_metrics.hpp"
#include "common_interfaces_cpp/webgui/MeasuringThreadingGUI.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"
#include "Frequency.hpp"

class WebGUI : public MeasuringThreadingGUI
{
public:
    WebGUI(const std::string& host = "127.0.0.1",
           const std::string& port = "2303",
           double freq = 20.0);
    ~WebGUI() override;

    static std::vector<double> getPose();
    static double getPerformance();

    std::vector<double> getRobotPose();
    double getCurrentPerformance();

    void setPerformance(double performance);

protected:
    void update_gui() override;

private:
    void setup_ros2();
    void performance_callback(const gazebo_msgs::msg::PerformanceMetrics::SharedPtr msg);

    std::shared_ptr<rclcpp::Node> bridge_node_;
    rclcpp::Subscription<gazebo_msgs::msg::PerformanceMetrics>::SharedPtr perf_sub_;

    std::shared_ptr<OdometryNode> odom_node_;

    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread executor_thread_;

    double performance_;
    std::mutex performance_mutex_;
};

extern std::shared_ptr<WebGUI> gui;

#endif