#include "WebGUI.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"
#include "common_interfaces_cpp/webgui/RTFMonitor.hpp"
#include "common_interfaces_cpp/webgui/WebGUIBridge.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include <atomic>
#include <cv_bridge/cv_bridge.h>
#include <mutex>

class WebGUINode : public BaseWebGUI {
public:
  WebGUINode()
      : BaseWebGUI("webgui_node", "127.0.0.1", "2303", 30.0),
        image_updated_(false),
        last_image_payload_("{\"image\":\"\",\"shape\":[]}"),
        gui_iterations_(0),
        rtf_monitor_("/stats", std::chrono::milliseconds(500)) {
    odom_node_ = std::make_shared<OdometryNode>("/f1/odom", "webgui_odom");
    rival_odom_node_ =
        std::make_shared<OdometryNode>("/f1_rival/odom", "webgui_rival_odom");
    aux_node_ = std::make_shared<rclcpp::Node>("webgui_aux");

    auto qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
    debug_sub_ = aux_node_->create_subscription<sensor_msgs::msg::Image>(
        "/webgui_image", qos,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
          try {
            cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
            show_image(img);
          } catch (...) {
          }
        });

    // Race status latched by the rival
    auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    status_sub_ = aux_node_->create_subscription<std_msgs::msg::String>(
        "/f1_pursuit/status", latched,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(status_mtx_);
          status_ = msg->data;
        });

    last_stat_time_ = std::chrono::steady_clock::now();
    stats_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&WebGUINode::send_stats, this));
  }

  std::vector<rclcpp::Node::SharedPtr> get_internal_nodes() {
    return {shared_from_this(), odom_node_, rival_odom_node_, aux_node_};
  }

  void show_image(const cv::Mat &img) {
    if (img.empty())
      return;
    std::lock_guard<std::mutex> lk(img_mtx_);
    img.copyTo(img_buf_);
    image_updated_.store(true);
  }

protected:
  json update_gui() override {
    gui_iterations_++;

    json inner;
    inner["image"] = encode_image();
    Pose3d pose = odom_node_->getPose3d();
    // Yaw goes along so the map can draw where each car points
    inner["map"] = "(" + std::to_string(pose.x) + ", " + std::to_string(pose.y) +
                   ", " + std::to_string(pose.yaw) + ")";
    Pose3d rival = rival_odom_node_->getPose3d();
    inner["rival"] = "(" + std::to_string(rival.x) + ", " +
                     std::to_string(rival.y) + ", " +
                     std::to_string(rival.yaw) + ")";
    {
      std::lock_guard<std::mutex> lk(status_mtx_);
      inner["pursuit"] = status_;
    }

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
    cv::imencode(".jpg", out, buf, {cv::IMWRITE_JPEG_QUALITY, 60});

    json p;
    p["image"] = base64_encode(buf.data(), buf.size());
    p["shape"] = std::vector<int>{out.rows, out.cols, 3};

    last_image_payload_ = p.dump();
    return last_image_payload_;
  }

  std::shared_ptr<OdometryNode> odom_node_;
  std::shared_ptr<OdometryNode> rival_odom_node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  std::mutex status_mtx_;
  std::string status_;
  rclcpp::Node::SharedPtr aux_node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr debug_sub_;
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

void WebGUI::show_image(const cv::Mat &image) {
  if (gui_node_)
    gui_node_->show_image(image);
}