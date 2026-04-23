#include "WebGUI.hpp"
#include "common_interfaces_cpp/webgui/WebGUIBridge.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "Lap.hpp"
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <string>
#include <vector>
#include <thread>

static std::string base64_encode(const unsigned char* data, size_t len) {
    static const char lookup[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve((len + 2) / 3 * 4);
    int i = 0, j = 0;
    unsigned char a3[3], a4[4];
    while (len--) {
        a3[i++] = *(data++);
        if (i == 3) {
            a4[0] = (a3[0] & 0xfc) >> 2;
            a4[1] = ((a3[0] & 0x03) << 4) + ((a3[1] & 0xf0) >> 4);
            a4[2] = ((a3[1] & 0x0f) << 2) + ((a3[2] & 0xc0) >> 6);
            a4[3] = a3[2] & 0x3f;
            for (i = 0; i < 4; i++) out += lookup[a4[i]];
            i = 0;
        }
    }
    if (i) {
        for (j = i; j < 3; j++) a3[j] = '\0';
        a4[0] = (a3[0] & 0xfc) >> 2;
        a4[1] = ((a3[0] & 0x03) << 4) + ((a3[1] & 0xf0) >> 4);
        a4[2] = ((a3[1] & 0x0f) << 2) + ((a3[2] & 0xc0) >> 6);
        a4[3] = a3[2] & 0x3f;
        for (j = 0; j < i + 1; j++) out += lookup[a4[j]];
        while (i++ < 3) out += '=';
    }
    return out;
}

class WebGUIImpl : public BaseWebGUI
{
public:
    WebGUIImpl()
        : BaseWebGUI("webgui_node", "127.0.0.1", "2303", 30.0, "/stats"),
          image_to_be_shown_updated_(false),
          auto_image_mode_(true)
    {
        odom_node_ = std::make_shared<OdometryNode>("/odom", "webgui_odom");
        lap_ = std::make_shared<Lap>(odom_node_);
        debug_node_ = std::make_shared<rclcpp::Node>("webgui_debug_node");
        
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
        
        debug_sub_ = debug_node_->create_subscription<sensor_msgs::msg::Image>(
            "/webgui_image", 
            qos,
            std::bind(&WebGUIImpl::debug_image_callback, this, std::placeholders::_1)
        );
    }

    std::vector<rclcpp::Node::SharedPtr> get_nodes() override
    {
        return {shared_from_this(), odom_node_, debug_node_};
    }

    void show_image(const cv::Mat& image)
    {
        if (image.empty()) return;
        std::lock_guard<std::mutex> lock(image_show_lock_);
        image_to_be_shown_ = image.clone();
        image_to_be_shown_updated_ = true;
    }

protected:
    json update_gui() override
    {
        json payload;
        payload["image"] = payloadImage().dump();
        payload["lap"] = lap_->check_threshold();

        Pose3d pose = odom_node_->getPose3d();
        payload["map"] = "(" + std::to_string(pose.x) + ", " + std::to_string(pose.y) + ")";

        return payload;
    }

    void process_message(const std::string& msg) override
    {
        if (msg.find("ack") != std::string::npos) {
            std::lock_guard<std::mutex> lock(ack_lock_);
            ack_ = true;
        } else if (msg.find("startLap") != std::string::npos) {
            lap_->unpause();
        } else if (msg.find("start") != std::string::npos) {
            std::lock_guard<std::mutex> lock(ack_lock_);
            ack_frontend_ = true;
        } else if (msg.find("pause") != std::string::npos) {
            lap_->pause();
        }
    }

private:
    json payloadImage()
    {
        cv::Mat local_img;
        
        {
            std::lock_guard<std::mutex> lock(image_show_lock_);
            if (!image_to_be_shown_updated_ || image_to_be_shown_.empty()) {
                return {{"image", ""}, {"shape", ""}};
            }
            local_img = image_to_be_shown_;
            image_to_be_shown_updated_ = false;
        }

        cv::Mat resized_img;
        if (local_img.cols > 640) {
            double scale = 640.0 / local_img.cols;
            cv::resize(local_img, resized_img, cv::Size(), scale, scale);
        } else {
            resized_img = local_img;
        }

        std::vector<uchar> buf;
        std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 60};
        cv::imencode(".JPEG", resized_img, buf, compression_params);
        std::string encoded = base64_encode(buf.data(), buf.size());

        json p;
        p["image"] = encoded;
        p["shape"] = std::vector<int>{resized_img.rows, resized_img.cols, 3};
        return p;
    }

    void debug_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
            show_image(img);
        } catch (const std::exception& e) {
        }
    }

    std::shared_ptr<OdometryNode> odom_node_;
    std::shared_ptr<Lap> lap_;
    rclcpp::Node::SharedPtr debug_node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr debug_sub_;
    cv::Mat image_to_be_shown_;
    bool image_to_be_shown_updated_;
    std::mutex image_show_lock_;
    bool auto_image_mode_;
};

namespace {
    std::shared_ptr<WebGUIImpl> gui_instance_ = nullptr;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_ = nullptr;
    std::thread spin_thread_;
}

void WebGUI::init()
{
    if (!gui_instance_) {
        gui_instance_ = std::make_shared<WebGUIImpl>();
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        
        for (const auto& node : gui_instance_->get_nodes()) {
            executor_->add_node(node);
        }

        spin_thread_ = std::thread([]() {
            executor_->spin();
        });
        spin_thread_.detach();
    }
}

void WebGUI::show_image(const cv::Mat& image)
{
    if (gui_instance_) gui_instance_->show_image(image);
}