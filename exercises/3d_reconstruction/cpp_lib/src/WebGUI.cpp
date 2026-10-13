#include "WebGUI.hpp"
#include "common_interfaces_cpp/webgui/WebGUIBridge.hpp"
#include "common_interfaces_cpp/webgui/RTFMonitor.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/empty.hpp"
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <atomic>
#include <deque>
#include <set>

class WebGUINode : public BaseWebGUI
{
public:
    WebGUINode()
        : BaseWebGUI("webgui_node", "127.0.0.1", "2303", 30.0)
        , images_updated_(false)
        , last_img1_payload_("{\"img\":\"\"}")
        , last_img2_payload_("{\"img\":\"\"}")
        , paint_matching_(false)
        , gui_iterations_(0)
        , rtf_monitor_("/stats", std::chrono::milliseconds(500))
    {
        aux_node_ = std::make_shared<rclcpp::Node>("webgui_aux");

        auto qos_transient = rclcpp::QoS(10).transient_local().reliable();
        auto qos_default   = rclcpp::QoS(10);

        // Left / right camera images for the 2D canvas
        img_left_sub_ = aux_node_->create_subscription<sensor_msgs::msg::Image>(
            "/webgui/image_left", qos_transient,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                try {
                    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
                    std::lock_guard<std::mutex> lk(img_mtx_);
                    img.copyTo(img1_buf_);
                    images_updated_.store(true);
                } catch (...) {}
            });

        img_right_sub_ = aux_node_->create_subscription<sensor_msgs::msg::Image>(
            "/webgui/image_right", qos_transient,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                try {
                    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
                    std::lock_guard<std::mutex> lk(img_mtx_);
                    img.copyTo(img2_buf_);
                    images_updated_.store(true);
                } catch (...) {}
            });

        // Toggle epipolar line drawing
        paint_match_sub_ = aux_node_->create_subscription<std_msgs::msg::Bool>(
            "/webgui/paint_matching", qos_default,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                paint_matching_.store(msg->data);
            });

        // New 3D points (deduplicated before adding to the send queue)
        pts_new_sub_ = aux_node_->create_subscription<std_msgs::msg::String>(
            "/webgui/points_new", qos_default,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                try {
                    auto pts = json::parse(msg->data);
                    std::vector<std::array<double, 6>> v;
                    for (const auto& p : pts) {
                        if (p.size() >= 6)
                            v.push_back({ p[0], p[1], p[2], p[3], p[4], p[5] });
                    }
                    add_new_points(v);
                } catch (...) {}
            });

        // Replace the entire point cloud
        pts_all_sub_ = aux_node_->create_subscription<std_msgs::msg::String>(
            "/webgui/points_all", qos_default,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                try {
                    auto pts = json::parse(msg->data);
                    std::vector<std::array<double, 6>> v;
                    for (const auto& p : pts) {
                        if (p.size() >= 6)
                            v.push_back({ p[0], p[1], p[2], p[3], p[4], p[5] });
                    }
                    set_all_points(v);
                } catch (...) {}
            });

        // Single matching line: [x1, y1, x2, y2]
        img_match_sub_ = aux_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/webgui/image_matching", qos_default,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() >= 4) {
                    std::lock_guard<std::mutex> lk(match_mtx_);
                    matching_.push_back({
                        static_cast<double>(msg->data[0]),
                        static_cast<double>(msg->data[1]),
                        static_cast<double>(msg->data[2]),
                        static_cast<double>(msg->data[3])
                    });
                }
            });

        // Full reset
        clear_sub_ = aux_node_->create_subscription<std_msgs::msg::Empty>(
            "/webgui/clear_points", qos_default,
            [this](const std_msgs::msg::Empty::SharedPtr) {
                do_clear();
            });

        last_stat_time_ = std::chrono::steady_clock::now();
        stats_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&WebGUINode::send_stats, this));
    }

    std::vector<rclcpp::Node::SharedPtr> get_internal_nodes() {
        return { shared_from_this(), aux_node_ };
    }

    // Public interface

    void show_images(const cv::Mat& left, const cv::Mat& right, bool paint_matching) {
        paint_matching_.store(paint_matching);
        std::lock_guard<std::mutex> lk(img_mtx_);
        left.copyTo(img1_buf_);
        right.copyTo(img2_buf_);
        images_updated_.store(true);
    }

    void add_new_points(const std::vector<std::array<double, 6>>& points) {
        std::lock_guard<std::mutex> lk(points_mtx_);
        for (const auto& p : points) {
            if (point_set_.insert(p).second)   // only add if not already present
                point_to_send_.push_back(p);
        }
    }

    void set_all_points(const std::vector<std::array<double, 6>>& points) {
        std::lock_guard<std::mutex> lk(points_mtx_);
        point_to_send_.clear();
        point_set_.clear();
        for (const auto& p : points) {
            point_set_.insert(p);
            point_to_send_.push_back(p);
        }
    }

    void add_matching(double x1, double y1, double x2, double y2) {
        std::lock_guard<std::mutex> lk(match_mtx_);
        matching_.push_back({ x1, y1, x2, y2 });
    }

    void do_clear() {
        {
            std::lock_guard<std::mutex> lk(points_mtx_);
            point_to_send_.clear();
            point_set_.clear();
        }
        {
            std::lock_guard<std::mutex> lk(match_mtx_);
            matching_.clear();
        }
        json reset;
        reset["reset"] = true;
        send_to_frontend(reset);
    }

protected:
    json update_gui() override {
        gui_iterations_++;

        auto [img1_str, img2_str] = encode_images();

        json inner;
        inner["img1"] = img1_str;
        inner["img2"] = img2_str;

        // Batch up to 500 points per tick to avoid oversized WebSocket frames.
        {
            std::lock_guard<std::mutex> lk(points_mtx_);
            size_t n = std::min(point_to_send_.size(), size_t(500));
            if (n > 0) {
                json pts = json::array();
                for (size_t i = 0; i < n; i++) {
                    json row = json::array();
                    for (double v : point_to_send_[i]) row.push_back(v);
                    pts.push_back(row);
                }
                inner["pts"] = pts.dump();
                point_to_send_.erase(point_to_send_.begin(), point_to_send_.begin() + n);
            } else {
                inner["pts"] = json::array().dump();
            }
        }

        {
            std::lock_guard<std::mutex> lk(match_mtx_);
            json match = json::array();
            for (const auto& m : matching_) {
                match.push_back({ m[0], m[1], m[2], m[3] });
            }
            inner["match"] = match.dump();
            matching_.clear();
        }

        inner["p_match"] = paint_matching_.load() ? "T" : "F";

        return inner;
    }

private:
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

    // Encodes one camera image as {"img": "base64..."} JSON string.
    // Returns {"img":""} when the image buffer is empty.
    std::string encode_one(const cv::Mat& img) {
        if (img.empty()) { return last_img1_payload_[0] == '{' ? "{\"img\":\"\"}" : "{\"img\":\"\"}"; }
        cv::Mat out;
        cv::resize(img, out, cv::Size(), 0.5, 0.5);
        std::vector<uchar> buf;
        cv::imencode(".jpg", out, buf, { cv::IMWRITE_JPEG_QUALITY, 60 });
        json p;
        p["img"] = base64_encode(buf.data(), buf.size());
        return p.dump();
    }

    // Returns a pair of JSON strings for both cameras.
    // If no new frame arrived since last call, returns the previous payloads.
    std::pair<std::string, std::string> encode_images() {
        cv::Mat local1, local2;
        {
            std::lock_guard<std::mutex> lk(img_mtx_);
            if (!images_updated_.load())
                return { last_img1_payload_, last_img2_payload_ };
            img1_buf_.copyTo(local1);
            img2_buf_.copyTo(local2);
            images_updated_.store(false);
        }
        last_img1_payload_ = encode_one(local1);
        last_img2_payload_ = encode_one(local2);
        return { last_img1_payload_, last_img2_payload_ };
    }

    rclcpp::Node::SharedPtr aux_node_;

    // ROS 2 subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr          img_left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr          img_right_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              paint_match_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr            pts_new_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr            pts_all_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr img_match_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr             clear_sub_;

    // Stereo image buffers (both protected by a single lock, updated together)
    cv::Mat img1_buf_, img2_buf_;
    std::mutex img_mtx_;
    std::atomic<bool> images_updated_;
    std::string last_img1_payload_;
    std::string last_img2_payload_;

    // 3D point cloud (deque for efficient front erasure during batching)
    std::deque<std::array<double, 6>> point_to_send_;
    std::set<std::array<double, 6>>   point_set_;   // deduplication
    std::mutex points_mtx_;

    // Epipolar matching lines: [x1, y1, x2, y2]
    std::vector<std::array<double, 4>> matching_;
    std::mutex match_mtx_;

    std::atomic<bool> paint_matching_;

    RTFMonitor rtf_monitor_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    std::atomic<int> gui_iterations_;
    std::chrono::time_point<std::chrono::steady_clock> last_stat_time_;
};

// Static member definitions
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

void WebGUI::show_images(const cv::Mat& left, const cv::Mat& right, bool paint_matching)
{
    if (gui_node_) gui_node_->show_images(left, right, paint_matching);
}

void WebGUI::show_new_points(const std::vector<std::array<double, 6>>& points)
{
    if (gui_node_) gui_node_->add_new_points(points);
}

void WebGUI::show_all_points(const std::vector<std::array<double, 6>>& points)
{
    if (gui_node_) gui_node_->set_all_points(points);
}

void WebGUI::show_image_matching(double x1, double y1, double x2, double y2)
{
    if (gui_node_) gui_node_->add_matching(x1, y1, x2, y2);
}

void WebGUI::clear_all_points()
{
    if (gui_node_) gui_node_->do_clear();
}
