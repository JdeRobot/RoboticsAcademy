#ifndef INCLUDE_WEBGUI_HPP_
#define INCLUDE_WEBGUI_HPP_

#include <opencv2/opencv.hpp>
#include <memory>
#include <thread>

// Forward declarations to avoid heavy ROS 2 includes
class WebGUINode;
namespace rclcpp::executors { class MultiThreadedExecutor; }

// show_image feeds the right panel and show_left_image the left one
// A ROS 2 direct solution can publish its images on /webgui/image_debug_right and
// /webgui/image_debug_left instead
class WebGUI
{
public:
    // Prevent instantiation. WebGUI acts as a global static utility.
    WebGUI() = delete;

    // Shows a BGR image in the right panel
    static void show_image(const cv::Mat& image);

    // Shows a BGR image in the left panel
    static void show_left_image(const cv::Mat& image);

private:
    static void init();
    friend class SystemBootstrapper;

    // Hidden internal state. Not accessible to the user.
    static std::shared_ptr<WebGUINode> gui_node_;
    static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    static std::thread spin_thread_;
};

#endif
