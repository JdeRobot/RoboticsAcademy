#include "WebGUI.hpp"
#include "Map.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"
#include "common_interfaces_cpp/webgui/RTFMonitor.hpp"
#include "common_interfaces_cpp/webgui/WebGUIBridge.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

// Nav2 map_server thresholds written in the exported YAML
static constexpr double OCCUPIED_THRESH = 0.65;
static constexpr double FREE_THRESH = 0.25;
// Unknown pixels kept around the mapped area of the exported map
static constexpr int CROP_MARGIN = 10;

class WebGUINode : public BaseWebGUI {
public:
  WebGUINode()
      : BaseWebGUI("webgui_node", "127.0.0.1", "2303", 30.0),
        image_updated_(false),
        last_image_payload_("{\"user_map\":null,\"shape\":0}"),
        gui_iterations_(0),
        rtf_monitor_("/stats", std::chrono::milliseconds(500)),
        map_util_([this]() { return odom_node_->getPose3d(); },
                  [this]() { return noisy_odom_node_->getPose3d(); }) {
    odom_node_ = std::make_shared<OdometryNode>("/do150/odom", "webgui_odom");
    noisy_odom_node_ = std::make_shared<OdometryNode>("/do150/odom_noisy",
                                                      "webgui_noisy_odom");
    aux_node_ = std::make_shared<rclcpp::Node>("webgui_aux");

    auto qos_transient =
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    user_map_sub_ = aux_node_->create_subscription<sensor_msgs::msg::Image>(
        "/webgui/user_map", qos_transient,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
          try {
            cv::Mat img = cv_bridge::toCvShare(msg, "mono8")->image;
            set_user_map(img);
          } catch (...) {
          }
        });

    last_stat_time_ = std::chrono::steady_clock::now();
    stats_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&WebGUINode::send_stats, this));
  }

  std::vector<rclcpp::Node::SharedPtr> get_internal_nodes() {
    return {shared_from_this(), odom_node_, noisy_odom_node_, aux_node_};
  }

  void set_user_map(const cv::Mat &img) {
    if (img.empty())
      return;

    if (img.rows != map_canvas::HEIGHT || img.cols != map_canvas::WIDTH) {
      throw std::invalid_argument(
          "map passed has the wrong dimensions, it has to be " +
          std::to_string(map_canvas::HEIGHT) + " pixels high and " +
          std::to_string(map_canvas::WIDTH) + " pixels wide");
    }

    cv::Mat processed_image;
    if (img.channels() == 1) {
      cv::cvtColor(img, processed_image, cv::COLOR_GRAY2BGR);
    } else {
      img.copyTo(processed_image);
    }

    std::lock_guard<std::mutex> lk(img_mtx_);
    processed_image.copyTo(img_buf_);
    image_updated_.store(true);
  }

  std::vector<double> pose_to_map(double x_prime, double y_prime,
                                  double yaw_prime) {
    auto [y, x] = map_canvas::world_to_pixel(x_prime, y_prime);
    double yaw = yaw_prime - CV_PI / 2.0;
    return {std::round(x), std::round(y), yaw};
  }

protected:
  json update_gui() override {
    gui_iterations_++;

    json inner;
    inner["user_map"] = encode_image();

    auto real_pos = map_util_.get_robot_coordinates();
    inner["real_pose"] = "(" + std::to_string(std::get<0>(real_pos)) + ", " +
                         std::to_string(std::get<1>(real_pos)) + ", " +
                         std::to_string(std::get<2>(real_pos)) + ")";

    auto noisy_pos = map_util_.get_robot_coordinates_with_noise();
    inner["noisy_pose"] = "(" + std::to_string(std::get<0>(noisy_pos)) + ", " +
                          std::to_string(std::get<1>(noisy_pos)) + ", " +
                          std::to_string(std::get<2>(noisy_pos)) + ")";

    // The map file goes out once per download request from the frontend
    inner["map_file"] = map_requested_.exchange(false) ? export_map() : "";

    return inner;
  }

  void on_frontend_message(const std::string &msg) override {
    if (msg.find("download_map") != std::string::npos)
      map_requested_.store(true);
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

  // Builds the map in the Nav2 map_server format cropped to the area the user
  // mapped
  std::string export_map() {
    cv::Mat gray;
    {
      std::lock_guard<std::mutex> lk(img_mtx_);
      if (img_buf_.empty())
        return "null";
      cv::cvtColor(img_buf_, gray, cv::COLOR_BGR2GRAY);
    }

    // Nav2 puts world x along the columns and world y upwards along the rows
    cv::Mat nav2_map;
    cv::flip(gray.t(), nav2_map, 0);

    // Pixels map_server reads as free or occupied, the rest is unknown
    cv::Mat known = (nav2_map < (1.0 - OCCUPIED_THRESH) * 255.0) |
                    (nav2_map > (1.0 - FREE_THRESH) * 255.0);
    if (cv::countNonZero(known) == 0)
      return "null";

    cv::Rect box = cv::boundingRect(known);
    int left = std::max(box.x - CROP_MARGIN, 0);
    int top = std::max(box.y - CROP_MARGIN, 0);
    int right = std::min(box.br().x - 1 + CROP_MARGIN, nav2_map.cols - 1);
    int bottom = std::min(box.br().y - 1 + CROP_MARGIN, nav2_map.rows - 1);
    cv::Mat cropped =
        nav2_map(cv::Rect(left, top, right - left + 1, bottom - top + 1));

    std::vector<uchar> pgm;
    cv::imencode(".pgm", cropped, pgm);

    // The origin is the world pose of the lower left corner of the crop
    // and pose_to_map rounds so every pixel is centred on its world point
    double origin_x = map_canvas::ORIGIN_X + (left - 0.5) / map_canvas::SCALE_X;
    double origin_y =
        map_canvas::ORIGIN_Y +
        (map_canvas::WIDTH - 1 - bottom - 0.5) / map_canvas::SCALE_Y;
    double resolution = 2.0 / (map_canvas::SCALE_X + map_canvas::SCALE_Y);

    char yaml[256];
    std::snprintf(yaml, sizeof(yaml),
                  "image: map.pgm\nmode: trinary\nresolution: %.6f\n"
                  "origin: [%.4f, %.4f, 0.0]\nnegate: 0\n"
                  "occupied_thresh: %.2f\nfree_thresh: %.2f\n",
                  resolution, origin_x, origin_y, OCCUPIED_THRESH, FREE_THRESH);

    json file;
    file["pgm"] = base64_encode(pgm.data(), pgm.size());
    file["yaml"] = yaml;
    return file.dump();
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

    std::vector<uchar> buf;
    cv::imencode(".jpg", local, buf, {cv::IMWRITE_JPEG_QUALITY, 60});

    json p;
    p["user_map"] = base64_encode(buf.data(), buf.size());
    p["shape"] = std::vector<int>{local.rows, local.cols, local.channels()};

    last_image_payload_ = p.dump();
    return last_image_payload_;
  }

  std::shared_ptr<OdometryNode> odom_node_;
  std::shared_ptr<OdometryNode> noisy_odom_node_;
  rclcpp::Node::SharedPtr aux_node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr user_map_sub_;

  cv::Mat img_buf_;
  std::mutex img_mtx_;
  std::atomic<bool> image_updated_;
  std::string last_image_payload_;
  std::atomic<bool> map_requested_{false};

  Map map_util_;
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

void WebGUI::set_user_map(const cv::Mat &image) {
  if (gui_node_)
    gui_node_->set_user_map(image);
}

std::vector<double> WebGUI::pose_to_map(double x_prime, double y_prime,
                                        double yaw_prime) {
  if (gui_node_)
    return gui_node_->pose_to_map(x_prime, y_prime, yaw_prime);
  return {0.0, 0.0, 0.0};
}