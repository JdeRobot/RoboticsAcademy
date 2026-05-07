#include "WebGUI.hpp"
#include "Map.hpp"
#include "common_interfaces_cpp/webgui/WebGUIBridge.hpp"
#include "common_interfaces_cpp/webgui/RTFMonitor.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include <cv_bridge/cv_bridge.h>
#include <nlohmann/json.hpp>
#include <mutex>
#include <atomic>
#include <cmath>

using json = nlohmann::json;

class WebGUINode : public BaseWebGUI
{
public:
    WebGUINode()
        : BaseWebGUI("webgui_node", "127.0.0.1", "2303", 30.0)
        , image_updated_(false)
        , last_image_payload_("{\"image\":\"\",\"shape\":[]}")
        , gui_iterations_(0)
        , rtf_monitor_("/stats", std::chrono::milliseconds(500))
        , lift_state_(false)
        , array_str_("[]")
        , map_util_([this](){ return odom_node_->getPose3d(); })
    {
        odom_node_ = std::make_shared<OdometryNode>("/amazon_robot/odom", "webgui_odom");
        aux_node_ = std::make_shared<rclcpp::Node>("webgui_aux");

        auto qos_transient = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        
        // Subscription to detect when the platform is lifted
        lift_sub_ = aux_node_->create_subscription<std_msgs::msg::Float64>(
            "/platform/cmd_vel", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                lift_state_.store(msg->data > 0.0);
            });

        // Optional topic subscription for the path
        path_sub_ = aux_node_->create_subscription<std_msgs::msg::String>(
            "/webgui/path", qos_transient,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(array_mtx_);
                array_str_ = msg->data;
            });

        image_sub_ = aux_node_->create_subscription<sensor_msgs::msg::Image>(
            "/webgui/image", qos_transient,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                try {
                    cv::Mat img;
                    if (msg->encoding == "mono8" || msg->encoding == "8UC1") {
                        img = cv_bridge::toCvShare(msg, "mono8")->image;
                    } else {
                        img = cv_bridge::toCvShare(msg, "bgr8")->image;
                    }
                    show_image(img);
                } catch (...) {}
            });

        last_stat_time_ = std::chrono::steady_clock::now();
        stats_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&WebGUINode::send_stats, this));
    }

    std::vector<rclcpp::Node::SharedPtr> get_internal_nodes() {
        return { shared_from_this(), odom_node_, aux_node_ };
    }

    void show_image(const cv::Mat& img) {
        if (img.empty()) return;
        cv::Mat processed = process_colors(img);
        std::lock_guard<std::mutex> lk(img_mtx_);
        processed.copyTo(img_buf_);
        image_updated_.store(true);
    }

    void show_path(const std::vector<std::vector<double>>& array) {
        // In C++, we use nlohmann::json directly instead of regular expressions like in Python
        json j_array = json::array();
        for (const auto& wp : array) {
            if (wp.size() >= 2) {
                j_array.push_back({wp[0] * 0.72, wp[1] * 0.545});
            }
        }
        
        std::lock_guard<std::mutex> lk(array_mtx_);
        array_str_ = j_array.dump(); // Generates a string in the format "[[x,y],[x,y]]"
    }

    cv::Mat get_map(const std::string& url) {
        return cv::imread(url, cv::IMREAD_COLOR);
    }

protected:
    json update_gui() override {
        gui_iterations_++;

        json inner;

        {
            std::lock_guard<std::mutex> lk(array_mtx_);
            inner["array"] = array_str_;
        }

        inner["liftState"] = lift_state_.load();

        auto coords = map_util_.getRobotCoordinates();
        double yaw = map_util_.getRobotAngle();
        // The frontend expects "(y, x, yaw)", which is exactly how we pack it here:
        inner["map"] = "(" + std::to_string(std::get<0>(coords)) + ", " + 
                       std::to_string(std::get<1>(coords)) + ", " + 
                       std::to_string(yaw) + ")";

        inner["image"] = encode_image();

        return inner;
    }

private:
    cv::Mat process_colors(const cv::Mat& image) {
        if (image.channels() == 3) return image.clone();

        cv::Mat colored_image = cv::Mat::zeros(image.size(), CV_8UC3);

        for (int y = 0; y < image.rows; ++y) {
            for (int x = 0; x < image.cols; ++x) {
                uchar val = image.at<uchar>(y, x);
                cv::Vec3b& color = colored_image.at<cv::Vec3b>(y, x);
                
                if (val < 128) {
                    color = cv::Vec3b(val * 2, val * 2, val * 2);
                } else {
                    switch (val) {
                        case 128: color = cv::Vec3b(0, 0, 255); break;     // red
                        case 129: color = cv::Vec3b(0, 165, 255); break;   // orange
                        case 130: color = cv::Vec3b(0, 255, 255); break;   // yellow
                        case 131: color = cv::Vec3b(0, 255, 0); break;     // green
                        case 132: color = cv::Vec3b(255, 0, 0); break;     // blue
                        case 133: color = cv::Vec3b(130, 0, 75); break;    // indigo
                        case 134: color = cv::Vec3b(211, 0, 148); break;   // violet
                        default: color = cv::Vec3b(0, 0, 0); break;
                    }
                }
            }
        }
        return colored_image;
    }

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
    rclcpp::Node::SharedPtr aux_node_;
    
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lift_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr path_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    std::atomic<bool> lift_state_;

    std::string array_str_;
    std::mutex array_mtx_;

    cv::Mat img_buf_;
    std::mutex img_mtx_;
    std::atomic<bool> image_updated_;
    std::string last_image_payload_;

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

void WebGUI::show_path(const std::vector<std::vector<double>>& array)
{
    if (gui_node_) gui_node_->show_path(array);
}

cv::Mat WebGUI::get_map(const std::string& url)
{
    if (gui_node_) return gui_node_->get_map(url);
    return cv::Mat();
}