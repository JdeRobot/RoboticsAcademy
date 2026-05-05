#ifndef INCLUDE_WEBGUI_HPP_
#define INCLUDE_WEBGUI_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <string>
#include <thread>
#include <array>

// Forward declarations to speed up compilation by avoiding heavy ROS 2 includes.
// - WebGUINode links to "common_interfaces_cpp/webgui/WebGUIBridge.hpp"
class WebGUINode;
namespace rclcpp::executors { class MultiThreadedExecutor; }

class WebGUI
{
public:
    // Prevent instantiation. WebGUI acts as a global static utility.
    WebGUI() = delete;

    static void set_image(const cv::Mat& image);
    static void show_position(double x, double y, double angle);
    static void show_particles(const std::vector<std::vector<double>>& particles);
    static std::vector<double> pose_to_map(double x_prime, double y_prime, double yaw_prime);
    static std::vector<double> map_to_pose(double map_x, double map_y, double map_yaw);
    static cv::Mat get_map(const std::string& url);
    static cv::Mat get_bgr_map(const std::string& url);

private:
    static void init();
    friend class SystemBootstrapper;

    // Hidden internal state. Not accessible to the user.
    static std::shared_ptr<WebGUINode> gui_node_;
    static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    static std::thread spin_thread_;
};

#endif