#include "WebGUI.hpp"
#include "common_interfaces_cpp/webgui/WebGUIBridge.hpp"
#include "common_interfaces_cpp/webgui/RTFMonitor.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <rcutils/logging.h>
#include <atomic>
#include <cmath>
#include <mutex>

class WebGUINode : public BaseWebGUI
{
public:
    WebGUINode()
        : BaseWebGUI("webgui_node", "127.0.0.1", "2303", 30.0)
        , image_updated_(false)
        , last_image_payload_("{\"image_right\":\"\",\"shape_right\":[]}")
        , gui_iterations_(0)
        , rtf_monitor_("/stats", std::chrono::milliseconds(500))
    {
        // ROS2 direct support: subscribe to /webgui_image so a user node that
        // only links WebGUI can display images by publishing to this topic.
        aux_node_ = std::make_shared<rclcpp::Node>("webgui_aux");
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
        debug_sub_ = aux_node_->create_subscription<sensor_msgs::msg::Image>(
            "/webgui_image", qos,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                try {
                    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
                    show_image(img);
                } catch (...) {}
            });

        last_stat_time_ = std::chrono::steady_clock::now();
    }

    std::vector<rclcpp::Node::SharedPtr> get_internal_nodes() {
        return { shared_from_this(), aux_node_ };
    }

    void show_image(const cv::Mat& img) {
        if (img.empty()) return;
        std::lock_guard<std::mutex> lk(img_mtx_);
        img.copyTo(img_buf_);
        image_updated_.store(true);
    }

protected:
    json update_gui() override {
        gui_iterations_++;

        json inner;
        inner["image_right"] = encode_image();

        // Piggyback stats every ~500 ms inside the single write path.
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - last_stat_time_;
        if (elapsed.count() >= 0.5) {
            double freq = (elapsed.count() > 0.0)
                          ? gui_iterations_.exchange(0) / elapsed.count() : 0.0;
            last_stat_time_ = now;

            inner["brain"] = std::round(freq * 10.0) / 10.0;
            inner["gui"]   = 30.0;
            inner["rtf"]   = rtf_monitor_.get();
            inner["fps"]   = -1.0;
            inner["lat"]   = -1.0;
        }

        return inner;
    }

    void on_frontend_message(const std::string&) override {}

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
        p["image_right"] = base64_encode(buf.data(), buf.size());
        p["shape_right"] = std::vector<int>{ out.rows, out.cols, 3 };

        last_image_payload_ = p.dump();
        return last_image_payload_;
    }

    rclcpp::Node::SharedPtr aux_node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr debug_sub_;
    cv::Mat img_buf_;
    std::mutex img_mtx_;
    std::atomic<bool> image_updated_;
    std::string last_image_payload_;

    RTFMonitor rtf_monitor_;
    std::atomic<int> gui_iterations_;
    std::chrono::time_point<std::chrono::steady_clock> last_stat_time_;
};

std::shared_ptr<WebGUINode> WebGUI::gui_node_  = nullptr;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> WebGUI::executor_ = nullptr;
std::thread WebGUI::spin_thread_;

void WebGUI::init()
{
    if (gui_node_) return;
    rcutils_logging_set_logger_level("WebGUI", RCUTILS_LOG_SEVERITY_ERROR);
    gui_node_ = std::make_shared<WebGUINode>();
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    for (auto& node : gui_node_->get_internal_nodes())
        executor_->add_node(node);
    spin_thread_ = std::thread([]() { executor_->spin(); });
    spin_thread_.detach();
}

void WebGUI::showImage(const cv::Mat& image)
{
    if (gui_node_) gui_node_->show_image(image);
}
