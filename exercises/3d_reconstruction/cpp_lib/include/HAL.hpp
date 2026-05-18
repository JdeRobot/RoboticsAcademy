#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include <opencv2/opencv.hpp>
#include <memory>
#include <thread>
#include <string>

// Forward declarations to speed up compilation by avoiding heavy ROS 2 includes.
class CameraNode;
namespace rclcpp::executors { class SingleThreadedExecutor; }

class HAL
{
public:
    // Prevent instantiation. HAL acts as a global static utility.
    HAL() = delete;

    // Raw stereo images
    static cv::Mat get_image_left();
    static cv::Mat get_image_right();

    // 2D coordinate conversions
    static cv::Vec3d grafic_to_optical(const std::string& lr, const cv::Vec3d& point2d);
    static cv::Vec3d optical_to_grafic(const std::string& lr, const cv::Vec3d& point2d);

    // Camera projection / backprojection
    static cv::Vec4d backproject(const std::string& lr, const cv::Vec3d& point2d);
    // project: 3D homogeneous point into 2D homogeneous point
    static cv::Vec3d project(const std::string& lr, cv::Vec4d& point3d);

    // Camera world position from extrinsics
    static cv::Vec3d get_camera_position(const std::string& lr);

    // Scale a reconstructed 3D point for the frontend 3D viewer
    static cv::Vec3d project_3d_scene(const cv::Vec4d& point3d);

private:
    static void init();
    friend class SystemBootstrapper;

    // Hidden internal state. Not accessible to the user.
    static std::shared_ptr<CameraNode> camera_left_;
    static std::shared_ptr<CameraNode> camera_right_;
    static std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    static std::thread spin_thread_;
};

#endif
