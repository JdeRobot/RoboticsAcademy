#include "WebGUI.hpp"
#include <cv_bridge/cv_bridge.h>

WebGUI* WebGUI::instance_ = nullptr;

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

WebGUI::WebGUI()
    : BaseWebGUI("webgui_node", "127.0.0.1", "2303", 30.0, "/stats"),
      image_to_be_shown_updated_(false),
      auto_image_mode_(true)
{
    instance_ = this;

    odom_node_ = std::make_shared<OdometryNode>("/odom", "webgui_odom");
    lap_ = std::make_shared<Lap>(odom_node_);

    debug_node_ = std::make_shared<rclcpp::Node>("webgui_debug_node");
    
    auto qos = rclcpp::QoS(10);
    debug_sub_ = debug_node_->create_subscription<sensor_msgs::msg::Image>(
        "/webgui_image", 
        qos,
        std::bind(&WebGUI::debug_image_callback, this, std::placeholders::_1)
    );
}

WebGUI::~WebGUI()
{
    if (instance_ == this) {
        instance_ = nullptr;
    }
}

std::vector<rclcpp::Node::SharedPtr> WebGUI::get_nodes()
{
    auto nodes = BaseWebGUI::get_nodes();
    nodes.push_back(odom_node_);
    nodes.push_back(debug_node_);
    return nodes;
}

void WebGUI::debug_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
        showImage(img);
    } catch (const std::exception& e) {
    }
}

void WebGUI::showImage(const cv::Mat& image)
{
    if (!instance_ || image.empty()) return;

    std::lock_guard<std::mutex> lock(instance_->image_show_lock_);
    instance_->image_to_be_shown_ = image.clone();
    instance_->image_to_be_shown_updated_ = true;
}

std::map<std::string, std::string> WebGUI::get_image_mode()
{
    std::map<std::string, std::string> mode;
    if (!instance_) {
        mode["auto_mode"] = "false";
        mode["topic_subscribed"] = "null";
        mode["manual_mode_available"] = "false";
        return mode;
    }

    mode["auto_mode"] = instance_->auto_image_mode_ ? "true" : "false";
    mode["topic_subscribed"] = instance_->auto_image_mode_ ? "/webgui_image" : "null";
    mode["manual_mode_available"] = "true";
    
    return mode;
}

void WebGUI::process_message(const std::string& msg)
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

json WebGUI::payloadImage()
{
    std::lock_guard<std::mutex> lock(image_show_lock_);
    if (!image_to_be_shown_updated_ || image_to_be_shown_.empty()) {
        return {{"image", ""}, {"shape", ""}};
    }

    std::vector<uchar> buf;
    cv::imencode(".JPEG", image_to_be_shown_, buf);
    std::string encoded = base64_encode(buf.data(), buf.size());

    json p;
    p["image"] = encoded;
    p["shape"] = std::vector<int>{image_to_be_shown_.rows, image_to_be_shown_.cols, 3};
    image_to_be_shown_updated_ = false;
    return p;
}

json WebGUI::update_gui()
{
    json payload;
    payload["image"] = payloadImage().dump();
    payload["lap"] = lap_->check_threshold();

    Pose3d pose = odom_node_->getPose3d();
    payload["map"] = "(" + std::to_string(pose.x) + ", " + std::to_string(pose.y) + ")";

    return payload;
}