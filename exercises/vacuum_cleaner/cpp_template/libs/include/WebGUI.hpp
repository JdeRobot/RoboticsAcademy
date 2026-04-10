#ifndef INCLUDE_WEBGUI_HPP_
#define INCLUDE_WEBGUI_HPP_

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>
#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "gazebo_msgs/msg/performance_metrics.hpp"
#include "Frequency.hpp"

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;
using json = nlohmann::json;

class WebGUI
{
public:
    WebGUI();
    static std::string img_payload;
};

class WebGUINode : public rclcpp::Node
{
public:
    WebGUINode();
    static std::vector<double> get_pose();
    static double get_performance();

private:
    void pose_callback(nav_msgs::msg::Odometry::UniquePtr msg);
    void performance_callback(gazebo_msgs::msg::PerformanceMetrics::UniquePtr msg);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<gazebo_msgs::msg::PerformanceMetrics>::SharedPtr perf_sub_;

    static nav_msgs::msg::Odometry last_odom;
    static gazebo_msgs::msg::PerformanceMetrics last_perf;
};

#endif