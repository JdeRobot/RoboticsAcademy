#include "WebGUI.hpp"
#include "common_interfaces_cpp/webgui/WebGUIBridge.hpp"
#include "common_interfaces_cpp/webgui/RTFMonitor.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include <cv_bridge/cv_bridge.h>
#include <rcutils/logging.h>
#include <mutex>
#include <atomic>
#include <cmath>

std::shared_ptr<WebGUINode> WebGUI::gui_node_  = nullptr;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> WebGUI::executor_ = nullptr;
std::thread WebGUI::spin_thread_;

// Published latched by the harvest manager of the world
static const char* SCORE_TOPIC = "/tomato_greenhouse/score";

class WebGUINode : public BaseWebGUI
{
public:
    WebGUINode()
        : BaseWebGUI("webgui_tomatoes_harvesting", "127.0.0.1", "2303", 30.0)
        , gui_iterations_(0)
        , rtf_monitor_("/stats", std::chrono::milliseconds(500))
    {
        aux_node_ = std::make_shared<rclcpp::Node>("webgui_tomatoes_harvesting_aux");

        // Debug images sent over ROS 2
        left_sub_ = aux_node_->create_subscription<sensor_msgs::msg::Image>(
            "/webgui/image_debug_left", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                try {
                    show_left_image(cv_bridge::toCvShare(msg, "bgr8")->image);
                } catch (...) {}
            });
        right_sub_ = aux_node_->create_subscription<sensor_msgs::msg::Image>(
            "/webgui/image_debug_right", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                try {
                    show_right_image(cv_bridge::toCvShare(msg, "bgr8")->image);
                } catch (...) {}
            });
        score_sub_ = aux_node_->create_subscription<std_msgs::msg::String>(
            SCORE_TOPIC, rclcpp::QoS(1).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(mtx_);
                score_ = msg->data;
            });

        last_stat_time_ = std::chrono::steady_clock::now();
        stats_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&WebGUINode::send_stats, this));
    }

    std::vector<rclcpp::Node::SharedPtr> get_internal_nodes() {
        return { shared_from_this(), aux_node_ };
    }

    void show_right_image(const cv::Mat& img) {
        if (img.empty()) return;
        std::lock_guard<std::mutex> lk(mtx_);
        img.copyTo(right_img_);
    }

    void show_left_image(const cv::Mat& img) {
        if (img.empty()) return;
        std::lock_guard<std::mutex> lk(mtx_);
        img.copyTo(left_img_);
    }

protected:
    // Only new images are sent, the frontend keeps showing the last one meanwhile
    json update_gui() override {
        gui_iterations_++;
        cv::Mat left, right;
        std::string score;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            left = left_img_;
            right = right_img_;
            left_img_ = cv::Mat();
            right_img_ = cv::Mat();
            score = score_;
        }
        json payload;
        payload["image_left"] = encode_image(left);
        payload["image_right"] = encode_image(right);
        payload["score"] = score;
        return payload;
    }

    void on_frontend_message(const std::string&) override {}

private:
    // Calculates and broadcasts frequency and performance stats
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

    // Same JSON string as the Python WebGUI, empty when there is no new image
    std::string encode_image(const cv::Mat& image) {
        if (image.empty()) return "";
        std::vector<uchar> jpeg;
        cv::imencode(".jpg", image, jpeg);
        json p;
        p["image"] = base64_encode(jpeg.data(), jpeg.size());
        p["shape"] = std::vector<int>{ image.rows, image.cols, image.channels() };
        return p.dump();
    }

    rclcpp::Node::SharedPtr aux_node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr score_sub_;

    cv::Mat left_img_;
    cv::Mat right_img_;
    std::string score_;
    std::mutex mtx_;

    std::atomic<int> gui_iterations_;
    RTFMonitor rtf_monitor_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    std::chrono::time_point<std::chrono::steady_clock> last_stat_time_;
};

void WebGUI::init()
{
    if (gui_node_) return;
    // Suppress write-cancel warnings that fire when no frontend is connected
    rcutils_logging_set_logger_level("WebGUI", RCUTILS_LOG_SEVERITY_ERROR);
    gui_node_ = std::make_shared<WebGUINode>();

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    for (auto& node : gui_node_->get_internal_nodes()) {
        executor_->add_node(node);
    }

    spin_thread_ = std::thread([]() { executor_->spin(); });
    spin_thread_.detach();
}

void WebGUI::show_image(const cv::Mat& image)
{
    if (gui_node_) gui_node_->show_right_image(image);
}

void WebGUI::show_left_image(const cv::Mat& image)
{
    if (gui_node_) gui_node_->show_left_image(image);
}
