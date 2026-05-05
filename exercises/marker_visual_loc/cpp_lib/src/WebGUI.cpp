#include "WebGUI.hpp"
#include "Map.hpp"
#include "common_interfaces_cpp/webgui/WebGUIBridge.hpp"
#include "common_interfaces_cpp/webgui/RTFMonitor.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <atomic>
#include <cmath>

namespace {
    double quat_to_yaw(double qw, double qx, double qy, double qz) {
        double rotate_za0 = 2.0 * (qx * qy + qw * qz);
        double rotate_za1 = qw * qw + qx * qx - qy * qy - qz * qz;
        if (rotate_za0 != 0.0 && rotate_za1 != 0.0) {
            return std::atan2(rotate_za0, rotate_za1);
        }
        return 0.0;
    }
}

class WebGUINode : public BaseWebGUI
{
public:
    WebGUINode()
        : BaseWebGUI("webgui_node", "127.0.0.1", "2303", 30.0)
        , image_updated_(false)
        , last_image_payload_("{\"image\":\"\",\"shape\":[]}")
        , estimate_updated_(false)
        , gui_iterations_(0)
        , rtf_monitor_("/stats", std::chrono::milliseconds(500))
        , map_util_([this](){ return odom_node_->getPose3d(); },
                    [this](){ return noisy_odom_node_->getPose3d(); })
    {
        odom_node_ = std::make_shared<OdometryNode>("/turtlebot3/odom", "webgui_odom");
        noisy_odom_node_ = std::make_shared<OdometryNode>("/turtlebot3/odom_noisy", "webgui_noisy_odom");
        aux_node_ = std::make_shared<rclcpp::Node>("webgui_aux");

        auto qos_transient = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

        image_sub_ = aux_node_->create_subscription<sensor_msgs::msg::Image>(
            "/webgui/image_debug", qos_transient,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                try {
                    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
                    show_image(img);
                } catch (...) {}
            });

        pose_sub_ = aux_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/webgui/estimated_pose", qos_transient,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                double x = msg->pose.position.x;
                double y = msg->pose.position.y;
                auto ori = msg->pose.orientation;
                double yaw = quat_to_yaw(ori.w, ori.x, ori.y, ori.z);
                show_estimated_pose(std::make_tuple(x, y, yaw));
            });

        last_stat_time_ = std::chrono::steady_clock::now();
        stats_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&WebGUINode::send_stats, this));
    }

    std::vector<rclcpp::Node::SharedPtr> get_internal_nodes() {
        return { shared_from_this(), odom_node_, noisy_odom_node_, aux_node_ };
    }

    void show_image(const cv::Mat& img) {
        if (img.empty()) return;
        std::lock_guard<std::mutex> lk(img_mtx_);
        img.copyTo(img_buf_);
        image_updated_.store(true);
    }

    void show_estimated_pose(const std::tuple<double, double, double>& pose) {
        double x = std::get<0>(pose);
        double y = std::get<1>(pose);
        double yaw = std::get<2>(pose);

        double scale_y = -85.0;
        double offset_y = -6.88;
        double ty = scale_y * (offset_y - y);

        double scale_x = 83.0;
        double offset_x = 8.0;
        double tx = scale_x * (offset_x - x);

        std::lock_guard<std::mutex> lk(pose_mtx_);
        estimated_pose_ = std::make_tuple(tx, ty, yaw);
        estimate_updated_.store(true);
    }

protected:
    json update_gui() override {
        gui_iterations_++;
        json inner;

        auto real_pos = map_util_.get_robot_coordinates();
        if (real_pos) {
            inner["real_pose"] = "(" + std::to_string(std::get<0>(*real_pos)) + ", " + 
                                 std::to_string(std::get<1>(*real_pos)) + ", " + 
                                 std::to_string(std::get<2>(*real_pos)) + ")";
        }

        auto noisy_pos = map_util_.get_robot_coordinates_with_noise();
        if (noisy_pos) {
            inner["noisy_pose"] = "(" + std::to_string(std::get<0>(*noisy_pos)) + ", " + 
                                  std::to_string(std::get<1>(*noisy_pos)) + ", " + 
                                  std::to_string(std::get<2>(*noisy_pos)) + ")";
        }

        if (estimate_updated_.load()) {
            std::lock_guard<std::mutex> lk(pose_mtx_);
            inner["estimate_pose"] = "(" + std::to_string(std::get<0>(estimated_pose_)) + ", " + 
                                     std::to_string(std::get<1>(estimated_pose_)) + ", " + 
                                     std::to_string(std::get<2>(estimated_pose_)) + ")";
        }

        inner["image"] = encode_image();

        return inner;
    }

private:
    void send_stats() {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - last_stat_time_;

        double current_freq = 0.0;
        if (elapsed.count() > 0.0) {
            current_freq = gui_iterations_.exchange(0) / elapsed.count();
        }
        last_stat_time_ = now;

        json stats;
        stats["brain"] = std::round(current_freq * 10.0) / 10.0;
        stats["gui"]   = 30.0;
        stats["rtf"]   = rtf_monitor_.get();
        stats["fps"]   = -1.0;
        stats["lat"]   = -1.0;

        send_to_frontend(stats);
    }

    std::string encode_image() {
        cv::Mat local;
        {
            std::lock_guard<std::mutex> lk(img_mtx_);
            if (!image_updated_.load() || img_buf_.empty())
                return last_image_payload_;

            img_buf_.copyTo(local);
            image_updated_.store(false);
        }

        cv::Mat out;
        if (local.cols > 640) {
            double scale = 640.0 / local.cols;
            cv::resize(local, out, cv::Size(), scale, scale);
        } else {
            out = local;
        }

        std::vector<uchar> buf;
        cv::imencode(".jpg", out, buf, { cv::IMWRITE_JPEG_QUALITY, 60 });

        json p;
        p["image"] = base64_encode(buf.data(), buf.size());
        p["shape"] = std::vector<int>{ out.rows, out.cols, out.channels() };

        last_image_payload_ = p.dump();
        return last_image_payload_;
    }

    std::shared_ptr<OdometryNode> odom_node_;
    std::shared_ptr<OdometryNode> noisy_odom_node_;
    rclcpp::Node::SharedPtr aux_node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    cv::Mat img_buf_;
    std::mutex img_mtx_;
    std::atomic<bool> image_updated_;
    std::string last_image_payload_;

    std::tuple<double, double, double> estimated_pose_;
    std::mutex pose_mtx_;
    std::atomic<bool> estimate_updated_;

    Map map_util_;
    RTFMonitor rtf_monitor_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    std::atomic<int> gui_iterations_;
    std::chrono::time_point<std::chrono::steady_clock> last_stat_time_;
};

std::shared_ptr<WebGUINode> WebGUI::gui_node_  = nullptr;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> WebGUI::executor_ = nullptr;
std::thread WebGUI::spin_thread_;

void WebGUI::init()
{
    if (gui_node_) return;
    gui_node_ = std::make_shared<WebGUINode>();
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    for (auto& node : gui_node_->get_internal_nodes())
        executor_->add_node(node);
    spin_thread_ = std::thread([]() { executor_->spin(); });
    spin_thread_.detach();
}

void WebGUI::show_image(const cv::Mat& image)
{
    if (gui_node_) gui_node_->show_image(image);
}

void WebGUI::show_estimated_pose(const std::tuple<double, double, double>& pose)
{
    if (gui_node_) gui_node_->show_estimated_pose(pose);
}