#ifndef INCLUDE_WEBGUI_HPP_
#define INCLUDE_WEBGUI_HPP_

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>
#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"
#include "common_interfaces_cpp/hal/laser.hpp"
#include "Map.hpp"
#include "Lap.hpp"

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;
using json = nlohmann::json;

class WebGUI {
public:
    WebGUI();
    static void showForces(const std::vector<double>& v1, const std::vector<double>& v2, const std::vector<double>& v3);
    static void showLocalTarget(const std::vector<double>& v);
    static std::shared_ptr<Target> getNextTarget();
    static void setTargetx(double x);
    static void setTargety(double y);
};

class WebGUINode : public rclcpp::Node {
public:
    WebGUINode();
    static std::shared_ptr<Map> map_;
    static std::shared_ptr<Lap> lap_;
    
    static std::shared_ptr<OdometryNode> pose3d_node_;
    static std::shared_ptr<LaserNode> laser_node_;

    void publish_current_target();

private:
    void target_reached_callback(std_msgs::msg::Bool::UniquePtr msg);
    
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_car_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_obs_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_avg_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_target_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_reached_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_current_target_;

    std::shared_ptr<Target> current_target_obj_;
};

#endif