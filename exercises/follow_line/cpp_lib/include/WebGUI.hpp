#ifndef INCLUDE_WEBGUI_HPP_
#define INCLUDE_WEBGUI_HPP_

#include "common_interfaces_cpp/webgui/WebGUIBridge.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "Lap.hpp"
#include <mutex>
#include <opencv2/opencv.hpp>
#include <map>
#include <string>

class WebGUI : public BaseWebGUI
{
public:
    WebGUI();
    ~WebGUI() override;

    json update_gui() override;
    void process_message(const std::string& msg) override;
    std::vector<rclcpp::Node::SharedPtr> get_nodes() override;

    static void showImage(const cv::Mat& image);
    static std::map<std::string, std::string> get_image_mode();

private:
    json payloadImage();
    void debug_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    std::shared_ptr<OdometryNode> odom_node_;
    std::shared_ptr<Lap> lap_;
    
    rclcpp::Node::SharedPtr debug_node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr debug_sub_;

    cv::Mat image_to_be_shown_;
    bool image_to_be_shown_updated_;
    std::mutex image_show_lock_;
    bool auto_image_mode_;

    static WebGUI* instance_;
};

#endif