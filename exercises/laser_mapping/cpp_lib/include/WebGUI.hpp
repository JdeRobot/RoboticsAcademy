#ifndef INCLUDE_WEBGUI_HPP_
#define INCLUDE_WEBGUI_HPP_

#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <opencv2/opencv.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>
#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;
using json = nlohmann::json;

struct Pose3D {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

class WebGUINode : public rclcpp::Node
{
public:
    WebGUINode();
    static Pose3D get_pose();
    static Pose3D get_noisy_pose();
    static void set_user_map(const cv::Mat& image);
    static cv::Mat get_user_map();

private:
    void user_map_callback(sensor_msgs::msg::Image::SharedPtr msg);
    void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
    void noisy_odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr user_map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr noisy_odom_sub_;

    static Pose3D last_pose_;
    static Pose3D last_noisy_pose_;
    static cv::Mat user_map_;
    static std::mutex map_mutex_;
};

class WebGUI
{
public:
    WebGUI();
    static void setUserMap(const cv::Mat& image);
    static std::vector<double> poseToMap(double x_prime, double y_prime, double yaw_prime);
    static std::string base64_encode(unsigned char const* bytes_to_encode, unsigned int in_len);
};

#endif