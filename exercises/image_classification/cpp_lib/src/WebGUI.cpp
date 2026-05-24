#include "WebGUI.hpp"
#include "common_interfaces_cpp/webgui/WebGUIBridge.hpp"
#include "common_interfaces_cpp/webgui/RTFMonitor.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <atomic>

namespace {
    // Standard base64 alphabet used to decode browser-sent "pick" frames.
    static const std::string kB64Chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    std::vector<uchar> base64_decode(const std::string& encoded)
    {
        std::vector<uchar> out;
        out.reserve(encoded.size() * 3 / 4);
        int val = 0, valb = -8;
        for (unsigned char c : encoded) {
            if (c == '=') break;
            auto pos = kB64Chars.find(static_cast<char>(c));
            if (pos == std::string::npos) continue;
            val = (val << 6) + static_cast<int>(pos);
            valb += 6;
            if (valb >= 0) {
                out.push_back(static_cast<uchar>((val >> valb) & 0xFF));
                valb -= 8;
            }
        }
        return out;
    }
}

class WebGUINode : public BaseWebGUI
{
public:
    WebGUINode()
        : BaseWebGUI("webgui_node", "127.0.0.1", "2303", 30.0)
        , image_updated_(false)
        , last_image_payload_("{\"image\":\"\",\"shape\":[]}")
        , has_received_img_(false)
        , gui_iterations_(0)
        , rtf_monitor_("/stats", std::chrono::milliseconds(500))
    {
        aux_node_ = std::make_shared<rclcpp::Node>("webgui_aux");

        // Receive processed images published to /webgui_image by user's ROS 2 code.
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
        webgui_image_sub_ = aux_node_->create_subscription<sensor_msgs::msg::Image>(
            "/webgui_image", qos,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                try {
                    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
                    show_image(img);
                } catch (...) {}
            });

        // Publish input frames (from browser) to /input/image_raw for ROS 2 subscribers.
        input_pub_ = aux_node_->create_publisher<sensor_msgs::msg::Image>(
            "/input/image_raw", rclcpp::QoS(10));

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

    cv::Mat get_image() {
        std::lock_guard<std::mutex> lk(input_mtx_);
        return input_buf_.clone();
    }

protected:
    json update_gui() override {
        gui_iterations_++;

        json inner;
        inner["image"] = encode_image();

        // Trigger the first pick-ack cycle when no frame has arrived yet.
        if (!has_received_img_.load()) {
            json ack;
            ack["ack_img"] = "ack";
            ack["time"]    = "";
            send_to_frontend(ack);
        }

        return inner;
    }

    void on_frontend_message(const std::string& msg) override {
        if (msg.rfind("pick", 0) == 0)
            handle_input_frame(msg);
    }

private:
    void handle_input_frame(const std::string& msg)
    {
        // Protocol: "pick" (4 bytes) + base64_image + timestamp (20 bytes)
        constexpr size_t PREFIX_SIZE = 4;
        constexpr size_t TIME_SIZE   = 20;
        if (msg.size() <= PREFIX_SIZE + TIME_SIZE) return;

        std::string b64 = msg.substr(PREFIX_SIZE, msg.size() - PREFIX_SIZE - TIME_SIZE);
        std::string ts  = msg.substr(msg.size() - TIME_SIZE);

        // Strip data URL prefix when the browser sends it inline.
        const std::string data_prefix = "data:image/jpeg;base64,";
        if (b64.rfind(data_prefix, 0) == 0)
            b64 = b64.substr(data_prefix.size());

        auto bytes = base64_decode(b64);
        if (bytes.empty()) return;

        cv::Mat img = cv::imdecode(bytes, cv::IMREAD_COLOR);
        if (img.empty()) return;

        {
            std::lock_guard<std::mutex> lk(input_mtx_);
            img.copyTo(input_buf_);
        }
        has_received_img_.store(true);

        publish_input(img);

        // Ack tells the frontend it can send the next frame.
        json ack;
        ack["ack_img"] = "ack";
        ack["time"]    = ts;
        send_to_frontend(ack);
    }

    void publish_input(const cv::Mat& img)
    {
        try {
            std_msgs::msg::Header header;
            auto msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
            input_pub_->publish(*msg);
        } catch (...) {}
    }

    void send_stats() {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - last_stat_time_;

        double current_freq = 0.0;
        if (elapsed.count() > 0.0)
            current_freq = gui_iterations_.exchange(0) / elapsed.count();
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

    rclcpp::Node::SharedPtr aux_node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr webgui_image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr input_pub_;

    // Output buffer: show_image() / /webgui_image → browser
    cv::Mat img_buf_;
    std::mutex img_mtx_;
    std::atomic<bool> image_updated_;
    std::string last_image_payload_;

    // Input buffer: browser "pick" frames → get_image()
    cv::Mat input_buf_;
    std::mutex input_mtx_;
    std::atomic<bool> has_received_img_;

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

cv::Mat WebGUI::get_image()
{
    if (gui_node_) return gui_node_->get_image();
    return cv::Mat();
}
