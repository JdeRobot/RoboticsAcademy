#include "WebGUI.hpp"
#include "common_interfaces_cpp/webgui/RTFMonitor.hpp"
#include "common_interfaces_cpp/webgui/WebGUIBridge.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <atomic>
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <mutex>

class WebGUINode : public BaseWebGUI {
public:
  WebGUINode()
      : BaseWebGUI("webgui_node", "127.0.0.1", "2303", 30.0),
        left_updated_(false), right_updated_(false),
        gui_iterations_(0),
        rtf_monitor_("/stats", std::chrono::milliseconds(500)) {
    aux_node_ = std::make_shared<rclcpp::Node>("webgui_aux");

    auto qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
    chaser_sub_ = aux_node_->create_subscription<sensor_msgs::msg::Image>(
        "/f1/camera/image_raw", qos,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
          try {
            show_left_image(cv_bridge::toCvShare(msg, "bgr8")->image);
          } catch (...) {
          }
        });
    rival_sub_ = aux_node_->create_subscription<sensor_msgs::msg::Image>(
        "/f1_rival/camera/image_raw", qos,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
          try {
            show_right_image(cv_bridge::toCvShare(msg, "bgr8")->image);
          } catch (...) {
          }
        });

    last_stat_time_ = std::chrono::steady_clock::now();
    stats_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&WebGUINode::send_stats, this));
  }

  std::vector<rclcpp::Node::SharedPtr> get_internal_nodes() {
    return {shared_from_this(), aux_node_};
  }

  void show_left_image(const cv::Mat &img) {
    if (img.empty())
      return;
    std::lock_guard<std::mutex> lk(img_mtx_);
    img.copyTo(left_buf_);
    left_updated_.store(true);
  }

  void show_right_image(const cv::Mat &img) {
    if (img.empty())
      return;
    std::lock_guard<std::mutex> lk(img_mtx_);
    img.copyTo(right_buf_);
    right_updated_.store(true);
  }

protected:
  json update_gui() override {
    gui_iterations_++;

    json inner;
    inner["image_left"] = encode(left_buf_, left_updated_, last_left_,
                                 "image_left", "shape_left");
    inner["image_right"] = encode(right_buf_, right_updated_, last_right_,
                                  "image_right", "shape_right");


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
    stats["gui"] = 30.0;
    stats["rtf"] = rtf_monitor_.get();
    stats["fps"] = -1.0;
    stats["lat"] = -1.0;

    send_to_frontend(stats);
  }

  std::string encode(const cv::Mat &buf, std::atomic<bool> &updated,
                     std::string &cache, const char *key_image,
                     const char *key_shape) {
    cv::Mat local;
    {
      std::lock_guard<std::mutex> lk(img_mtx_);
      if (!updated.load() || buf.empty()) {
        if (cache.empty()) {
          json empty;
          empty[key_image] = nullptr;
          empty[key_shape] = 0;
          cache = empty.dump();
        }
        return cache;
      }
      buf.copyTo(local);
      updated.store(false);
    }

    cv::Mat out;
    if (local.cols > 640) {
      double scale = 640.0 / local.cols;
      cv::resize(local, out, cv::Size(), scale, scale);
    } else {
      out = local;
    }

    std::vector<uchar> enc;
    cv::imencode(".jpg", out, enc, {cv::IMWRITE_JPEG_QUALITY, 60});

    json p;
    p[key_image] = base64_encode(enc.data(), enc.size());
    p[key_shape] = std::vector<int>{out.rows, out.cols, 3};
    cache = p.dump();
    return cache;
  }

  rclcpp::Node::SharedPtr aux_node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr chaser_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rival_sub_;
  cv::Mat left_buf_;
  cv::Mat right_buf_;
  std::mutex img_mtx_;
  std::atomic<bool> left_updated_;
  std::atomic<bool> right_updated_;
  std::string last_left_;
  std::string last_right_;

  RTFMonitor rtf_monitor_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  std::atomic<int> gui_iterations_;
  std::chrono::time_point<std::chrono::steady_clock> last_stat_time_;
};

std::shared_ptr<WebGUINode> WebGUI::gui_node_ = nullptr;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> WebGUI::executor_ =
    nullptr;
std::thread WebGUI::spin_thread_;

void WebGUI::init() {
  if (gui_node_)
    return;
  gui_node_ = std::make_shared<WebGUINode>();
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  for (auto &node : gui_node_->get_internal_nodes())
    executor_->add_node(node);
  spin_thread_ = std::thread([]() { executor_->spin(); });
  spin_thread_.detach();
}

// The panels show the two cameras, there is no free half to draw on
void WebGUI::show_image(const cv::Mat &) {}

void WebGUI::show_left_image(const cv::Mat &) {}

void WebGUI::show_right_image(const cv::Mat &) {}