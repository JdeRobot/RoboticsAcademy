#ifndef INCLUDE_WEBGUI_HPP_
#define INCLUDE_WEBGUI_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <string>
#include <thread>

class WebGUINode;
namespace rclcpp::executors { class MultiThreadedExecutor; }

class WebGUI
{
public:
    // Prevent instantiation. WebGUI acts as a global static utility.
    WebGUI() = delete;

    static void show_image(const cv::Mat& image);
    static void show_path(const std::vector<std::vector<double>>& array);
    static cv::Mat get_map(const std::string& url);

private:
    static void init();
    friend class SystemBootstrapper;

    // Hidden internal state. Not accessible to the user.
    static std::shared_ptr<WebGUINode> gui_node_;
    static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    static std::thread spin_thread_;
};

#endif