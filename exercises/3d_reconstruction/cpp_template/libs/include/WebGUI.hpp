#ifndef INCLUDE_WEBGUI_HPP_
#define INCLUDE_WEBGUI_HPP_

#include <opencv2/opencv.hpp>
#include <memory>
#include <thread>
#include <vector>
#include <array>

// Forward declarations to speed up compilation by avoiding heavy ROS 2 includes.
class WebGUINode;
namespace rclcpp::executors { class MultiThreadedExecutor; }

class WebGUI
{
public:
    // Prevent instantiation. WebGUI acts as a global static utility.
    WebGUI() = delete;

    // Show the stereo image pair; paint_matching draws epipolar lines when true.
    static void show_images(const cv::Mat& left, const cv::Mat& right, bool paint_matching);

    // 3D point cloud management. Each point is [x, y, z, r, g, b].
    static void show_new_points(const std::vector<std::array<double, 6>>& points);
    static void show_all_points(const std::vector<std::array<double, 6>>& points);

    // Add an epipolar matching line between image coordinates (x1,y1) ↔ (x2,y2).
    static void show_image_matching(double x1, double y1, double x2, double y2);

    // Clear all points and reset the 3D viewer.
    static void clear_all_points();

private:
    static void init();
    friend class SystemBootstrapper;

    // Hidden internal state. Not accessible to the user.
    static std::shared_ptr<WebGUINode> gui_node_;
    static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    static std::thread spin_thread_;
};

#endif
