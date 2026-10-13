#ifndef INCLUDE_WEBGUI_HPP_
#define INCLUDE_WEBGUI_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <string>
#include <thread>
#include <array>

class WebGUINode;
namespace rclcpp::executors { class MultiThreadedExecutor; }

class WebGUI
{
public:
    WebGUI() = delete;

    static void show_position(double x, double y, double angle);
    static void show_particles(const std::vector<std::vector<double>>& particles);
    static std::vector<double> pose_to_map(double x_prime, double y_prime, double yaw_prime);
    static std::vector<double> map_to_pose(double map_x, double map_y, double map_yaw);
    static cv::Mat get_map(const std::string& url);

private:
    static void init();
    friend class SystemBootstrapper;

    static std::shared_ptr<WebGUINode> gui_node_;
    static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    static std::thread spin_thread_;
};

#endif