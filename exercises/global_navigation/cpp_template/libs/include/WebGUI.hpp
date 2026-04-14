#ifndef INCLUDE_WEBGUI_HPP_
#define INCLUDE_WEBGUI_HPP_

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"
#include "Map.hpp"

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;
using json = nlohmann::json;

class WebGUI
{
public:
    WebGUI();

    static void showNumpy(const cv::Mat& image);
    static void showPath(const std::vector<std::vector<int>>& array);
    static std::vector<double> getTargetPose();
    static cv::Mat getMap(const std::string& url);
    static std::vector<int> rowColumn(const std::vector<double>& pose);
    static std::vector<int> worldToGrid(const std::vector<double>& pose);
    static std::vector<double> gridToWorld(const std::vector<int>& cell);
    static void reset_gui();
    static std::string payloadImage();

    static std::vector<double> worldXY;
    static std::string array_str;

private:
    static cv::Mat image_to_be_shown;
    static bool image_to_be_shown_updated;
    static std::mutex image_show_lock;
    static std::mutex array_lock;
};

class WebGUINode : public rclcpp::Node
{
public:
    WebGUINode();
    
    static std::shared_ptr<Map> map_;
    static std::shared_ptr<OdometryNode> pose3d_node_;
    
    static void publish_target(double x, double y);

private:
    void path_callback(nav_msgs::msg::Path::UniquePtr msg);
    void image_callback(sensor_msgs::msg::Image::UniquePtr msg);

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    static std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Point>> target_pub_;
};

#endif