#ifndef WEBGUI_HPP_
#define WEBGUI_HPP_

#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <tuple>
#include <vector>
#include <functional>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "common_interfaces_cpp/webgui/MeasuringThreadingGUI.hpp"
#include "Map.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"
#include "common_interfaces_cpp/hal/camera.hpp"

std::string base64_encode(unsigned char const* bytes_to_encode, unsigned int in_len);

class WebGUI : public MeasuringThreadingGUI {
public:
    WebGUI(const std::string& host = "127.0.0.1", const std::string& port = "2303", double freq = 30.0);
    ~WebGUI() override;

    static void showImage(const cv::Mat& image);
    static void showEstimatedPose(const std::tuple<double, double, double>& pose);

    void setImage(const cv::Mat& image);
    void setEstimatedRobotPose(const std::tuple<double, double, double>& pose);

    Pose3d get_pose3d();
    Pose3d get_odom();

protected:
    void update_gui() override;

private:
    void setup_ros2();
    void estimated_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    cv::Mat image_;
    std::mutex image_lock_;
    
    std::tuple<double, double, double> predict_pose_;
    bool has_predict_pose_;

    std::shared_ptr<rclcpp::Node> bridge_node_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr estimated_pose_sub_;
    
    std::shared_ptr<OdometryNode> real_odom_node_;
    std::shared_ptr<OdometryNode> noisy_odom_node_;
    std::shared_ptr<CameraNode> camera_node_;

    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread executor_thread_;

    std::shared_ptr<Map> map_;
};

extern std::shared_ptr<WebGUI> gui;

#endif