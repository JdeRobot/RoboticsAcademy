#include "WebGUI.hpp"
#include "common_interfaces_cpp/webgui/WebGUIBridge.hpp"
#include "common_interfaces_cpp/webgui/RTFMonitor.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <atomic>
#include <cmath>

// Models this exercise allows the gripper to grab. Published from WebGUI (not
// HAL) so it also works in ROS2-direct mode, which imports WebGUI but not HAL.
static const char* GRASPABLE_MODELS = "package_box_01";

std::shared_ptr<WebGUINode> WebGUI::gui_node_  = nullptr;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> WebGUI::executor_ = nullptr;
std::thread WebGUI::spin_thread_;

class WebGUINode : public BaseWebGUI
{
public:
    WebGUINode()
        : BaseWebGUI("webgui_node_drone", "127.0.0.1", "2303", 30.0)
        , right_updated_(false)
        , left_updated_(false)
        , last_right_payload_("{\"image_right\":\"\",\"shape_right\":[]}")
        , last_left_payload_("{\"image_left\":\"\",\"shape_left\":[]}")
        , gui_iterations_(0)
        , rtf_monitor_("/stats", std::chrono::milliseconds(500))
    {
        aux_node_ = std::make_shared<rclcpp::Node>("webgui_aux");

        // QoS profile ensuring delivery of the latest image to late joiners
        auto qos_transient = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

        right_sub_ = aux_node_->create_subscription<sensor_msgs::msg::Image>(
            "/webgui/image_debug_right", qos_transient,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                try {
                    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
                    show_right_image(img);
                } catch (...) {}
            });

        left_sub_ = aux_node_->create_subscription<sensor_msgs::msg::Image>(
            "/webgui/image_debug_left", qos_transient,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                try {
                    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
                    show_left_image(img);
                } catch (...) {}
            });

        // Tell the gripper plugin which models are grabbable for this exercise.
        // Latched so the plugin receives the list even if it subscribes later,
        // and re-published on a timer so a plugin instance re-created by a reset
        // also receives it (otherwise it would grab nothing after a reset).
        auto qos_latched = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        graspable_pub_ = aux_node_->create_publisher<std_msgs::msg::String>(
            "/drone0/gripper/graspable", qos_latched);
        publish_graspable();
        graspable_timer_ = aux_node_->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&WebGUINode::publish_graspable, this));

        last_stat_time_ = std::chrono::steady_clock::now();
        stats_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&WebGUINode::send_stats, this));
    }

    std::vector<rclcpp::Node::SharedPtr> get_internal_nodes() {
        return { shared_from_this(), aux_node_ };
    }

    // Thread-safe image updates
    void show_right_image(const cv::Mat& img) {
        if (img.empty()) return;
        std::lock_guard<std::mutex> lk(right_mtx_);
        img.copyTo(right_img_buf_);
        right_updated_.store(true);
    }

    void show_left_image(const cv::Mat& img) {
        if (img.empty()) return;
        std::lock_guard<std::mutex> lk(left_mtx_);
        img.copyTo(left_img_buf_);
        left_updated_.store(true);
    }

protected:
    // Generates the JSON payload sent to the frontend
    json update_gui() override {
        gui_iterations_++;
        json inner;
        inner["image_right"] = encode_right_image();
        inner["image_left"]  = encode_left_image();
        return inner;
    }

private:
    // Publishes the grabbable-models list for the gripper plugin.
    void publish_graspable() {
        std_msgs::msg::String msg;
        msg.data = GRASPABLE_MODELS;
        graspable_pub_->publish(msg);
    }

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

    // Encodes the right image as a base64 string
    std::string encode_right_image() {
        cv::Mat local;
        {
            std::lock_guard<std::mutex> lk(right_mtx_);
            if (!right_updated_.load() || right_img_buf_.empty())
                return last_right_payload_;

            right_img_buf_.copyTo(local);
            right_updated_.store(false);
        }

        // Downscale image if too large to save bandwidth
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
        p["image_right"] = base64_encode(buf.data(), buf.size());
        p["shape_right"] = std::vector<int>{ out.rows, out.cols, out.channels() };

        last_right_payload_ = p.dump();
        return last_right_payload_;
    }

    // Encodes the left image as a base64 string
    std::string encode_left_image() {
        cv::Mat local;
        {
            std::lock_guard<std::mutex> lk(left_mtx_);
            if (!left_updated_.load() || left_img_buf_.empty())
                return last_left_payload_;

            left_img_buf_.copyTo(local);
            left_updated_.store(false);
        }

        // Downscale image if too large to save bandwidth
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
        p["image_left"] = base64_encode(buf.data(), buf.size());
        p["shape_left"] = std::vector<int>{ out.rows, out.cols, out.channels() };

        last_left_payload_ = p.dump();
        return last_left_payload_;
    }

    rclcpp::Node::SharedPtr aux_node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr graspable_pub_;
    rclcpp::TimerBase::SharedPtr graspable_timer_;

    cv::Mat right_img_buf_;
    cv::Mat left_img_buf_;
    std::mutex right_mtx_;
    std::mutex left_mtx_;
    std::atomic<bool> right_updated_;
    std::atomic<bool> left_updated_;
    
    std::string last_right_payload_;
    std::string last_left_payload_;

    RTFMonitor rtf_monitor_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    std::atomic<int> gui_iterations_;
    std::chrono::time_point<std::chrono::steady_clock> last_stat_time_;
};

void WebGUI::init()
{
    if (gui_node_) return;
    
    gui_node_ = std::make_shared<WebGUINode>();
    
    // MultiThreadedExecutor spins nodes concurrently
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    for (auto& node : gui_node_->get_internal_nodes()) {
        executor_->add_node(node);
    }
    
    // Detached background thread to keep executor spinning
    spin_thread_ = std::thread([]() { executor_->spin(); });
    spin_thread_.detach();
}

void WebGUI::show_right_image(const cv::Mat& image)
{
    if (gui_node_) gui_node_->show_right_image(image);
}

void WebGUI::show_left_image(const cv::Mat& image)
{
    if (gui_node_) gui_node_->show_left_image(image);
}