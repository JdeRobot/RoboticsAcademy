#ifndef INCLUDE_WEBGUI_HPP_
#define INCLUDE_WEBGUI_HPP_

#include <opencv2/opencv.hpp>
#include <memory>
#include <thread>
#include <tuple>

class WebGUINode;
namespace rclcpp::executors { class MultiThreadedExecutor; }

class WebGUI
{
public:
    WebGUI() = delete;

    static void show_image(const cv::Mat& image);
    static void show_estimated_pose(const std::tuple<double, double, double>& pose);

private:
    static void init();
    friend class SystemBootstrapper;

    static std::shared_ptr<WebGUINode> gui_node_;
    static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    static std::thread spin_thread_;
};

#endif