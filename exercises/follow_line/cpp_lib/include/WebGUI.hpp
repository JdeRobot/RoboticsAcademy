#ifndef INCLUDE_WEBGUI_HPP_
#define INCLUDE_WEBGUI_HPP_

#include <opencv2/opencv.hpp>
#include <memory>
#include <thread>

// Forward declaration to keep the header clean of Boost/JSON dependencies.
// - WebGUINode is internally implemented in WebGUI.cpp
class WebGUINode;
namespace rclcpp::executors { class MultiThreadedExecutor; }

class WebGUI
{
public:
    // Prevent instantiation. WebGUI acts as a global static utility.
    WebGUI() = delete;

    static void show_image(const cv::Mat& image);

private:
    static void init();
    friend class SystemBootstrapper;

    // Internal GUI state. Not accessible to the user.
    static std::shared_ptr<WebGUINode> gui_node_;
    static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    static std::thread spin_thread_;
};

#endif