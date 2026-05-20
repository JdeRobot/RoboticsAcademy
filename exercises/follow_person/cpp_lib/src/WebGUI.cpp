#include "WebGUI.hpp"
#include "common_interfaces_cpp/webgui/WebGUIBridge.hpp"
#include "common_interfaces_cpp/webgui/RTFMonitor.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <atomic>
#include <cmath>

class WebGUINode : public BaseWebGUI
{
public:
    WebGUINode()
        : BaseWebGUI("webgui_person_control", "127.0.0.1", "2303", 30.0)
        , image_updated_(false)
        , last_image_payload_("{\"image\":\"\",\"shape\":[]}")
        , gui_iterations_(0)
        , rtf_monitor_("/stats", std::chrono::milliseconds(500))
    {
        aux_node_ = std::make_shared<rclcpp::Node>("webgui_person_aux");

        person_cmd_pub_ = aux_node_->create_publisher<geometry_msgs::msg::Twist>(
            "/person/cmd_vel", 10);

        auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .best_effort()
            .durability_volatile();

        image_sub_ = aux_node_->create_subscription<sensor_msgs::msg::Image>(
            "/webgui/image_show", image_qos,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                try {
                    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
                    show_image(img);
                } catch (...) {}
            });

        last_stat_time_ = std::chrono::steady_clock::now();
        stats_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&WebGUINode::send_stats, this));
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
        inner["image"] = encode_image();

        return inner;
    }

    void on_frontend_message(const std::string& msg) override {
        geometry_msgs::msg::Twist twist;

        const double linear_speed = 0.01;
        const double angular_speed = 0.01;

        if (msg == "key_w") {
            twist.linear.x = linear_speed;
        } else if (msg == "key_s") {
            twist.linear.x = -linear_speed;
        } else if (msg == "key_w_up" || msg == "key_s_up") {
            twist.linear.x = 0.0;
        }

        if (msg == "key_a") {
            twist.angular.z = angular_speed;
        } else if (msg == "key_d") {
            twist.angular.z = -angular_speed;
        } else if (msg == "key_a_up" || msg == "key_d_up") {
            twist.angular.z = 0.0;
        }

        if (person_cmd_pub_) {
            person_cmd_pub_->publish(twist);
        }
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
        stats["gui"] = 30.0;
        stats["rtf"] = rtf_monitor_.get();
        stats["fps"] = -1.0;
        stats["lat"] = -1.0;

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
        cv::imencode(".png", out, buf);

        json p;
        p["image"] = base64_encode(buf.data(), buf.size());
        p["shape"] = std::vector<int>{ out.rows, out.cols, out.channels() };

        last_image_payload_ = p.dump();
        return last_image_payload_;
    }

    rclcpp::Node::SharedPtr aux_node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr person_cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    cv::Mat img_buf_;
    std::mutex img_mtx_;
    std::atomic<bool> image_updated_;
    std::string last_image_payload_;

    RTFMonitor rtf_monitor_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    std::atomic<int> gui_iterations_;
    std::chrono::time_point<std::chrono::steady_clock> last_stat_time_;
};

std::shared_ptr<WebGUINode> WebGUI::gui_node_ = nullptr;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> WebGUI::executor_ = nullptr;
std::thread WebGUI::spin_thread_;

void WebGUI::init()
{
    if (gui_node_) return;

    gui_node_ = std::make_shared<WebGUINode>();
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    for (auto& node : gui_node_->get_internal_nodes())
        executor_->add_node(node);

    spin_thread_ = std::thread([]() {
        executor_->spin();
    });

    spin_thread_.detach();
}

void WebGUI::show_image(const cv::Mat& image)
{
    if (gui_node_) gui_node_->show_image(image);
}