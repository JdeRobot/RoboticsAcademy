#include "WebGUI.hpp"
#include "Map.hpp"
#include "common_interfaces_cpp/webgui/WebGUIBridge.hpp"
#include "common_interfaces_cpp/webgui/RTFMonitor.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <atomic>
#include <cmath>

double quat_to_yaw(const geometry_msgs::msg::Quaternion& q) {
    double rotate_za0 = 2.0 * (q.x * q.y + q.w * q.z);
    double rotate_za1 = q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z;
    if (rotate_za0 != 0.0 && rotate_za1 != 0.0) {
        return std::atan2(rotate_za0, rotate_za1);
    }
    return 0.0;
}

class WebGUINode : public BaseWebGUI
{
public:
    WebGUINode()
        : BaseWebGUI("webgui_node", "127.0.0.1", "2303", 30.0)
        , image_updated_(false)
        , last_image_payload_("{\"image\":\"\",\"shape\":[]}")
        , gui_iterations_(0)
        , rtf_monitor_("/stats", std::chrono::milliseconds(500))
        , user_pose_({0.0, 0.0, 0.0})
        , map_util_([this](){ 
            return odom_node_->getPose3d(); 
          })
    {
        odom_node_ = std::make_shared<OdometryNode>("/vacuum_cleaner/odom", "webgui_odom");
        aux_node_ = std::make_shared<rclcpp::Node>("webgui_aux");

        auto qos_transient = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        auto qos_image = rclcpp::QoS(rclcpp::KeepLast(10));

        estimated_pose_sub_ = aux_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/webgui/estimated_pose", qos_transient,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                double yaw = quat_to_yaw(msg->pose.orientation);
                this->show_position(msg->pose.position.x, msg->pose.position.y, yaw);
            });

        particles_sub_ = aux_node_->create_subscription<geometry_msgs::msg::PoseArray>(
            "/webgui/particles", qos_transient,
            [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
                std::vector<std::vector<double>> pts;
                for (const auto& pose : msg->poses) {
                    double yaw = quat_to_yaw(pose.orientation);
                    pts.push_back({pose.position.x, pose.position.y, yaw});
                }
                this->show_particles(pts);
            });

        debug_sub_ = aux_node_->create_subscription<sensor_msgs::msg::Image>(
            "/webgui/image_debug", qos_image,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                try {
                    cv::Mat img = cv_bridge::toCvShare(msg, "passthrough")->image;
                    this->set_image(img);
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

    void set_image(const cv::Mat& img) {
        if (img.empty()) return;
        std::lock_guard<std::mutex> lk(img_mtx_);
        img.copyTo(img_buf_);
        image_updated_.store(true);
    }

    void show_position(double x, double y, double angle) {
        std::lock_guard<std::mutex> lk(user_mtx_);
        user_pose_ = { -30.0 * x + 171.0, 15.0 * y + 63.0, angle };
    }

    void show_particles(const std::vector<std::vector<double>>& particles) {
        std::lock_guard<std::mutex> lk(particles_mtx_);
        particles_.clear();
        for (const auto& p : particles) {
            if (p.size() >= 3) {
                double px = -30.0 * p[0] + 171.0;
                double py = 15.0 * p[1] + 63.0;
                double pyaw = p[2];
                particles_.push_back({px, py, pyaw});
            }
        }
    }

    cv::Mat get_map(const std::string& url) {
        return cv::imread(url);
    }

    cv::Mat get_bgr_map(const std::string& url) {
        return cv::imread(url, cv::IMREAD_COLOR);
    }

protected:
    json update_gui() override {
        gui_iterations_++;
        json inner;

        inner["image"] = encode_image();

        auto real_pos = map_util_.getRobotCoordinates();
        if (std::abs(real_pos[0] - 171.0) < 1e-3 && std::abs(real_pos[1] - 63.0) < 1e-3) {
            real_pos[0] = 201.0;
            real_pos[1] = 85.5;
        }
        double real_yaw = map_util_.getRobotAngle();
        inner["map"] = "(" + std::to_string(real_pos[0]) + ", " + 
                       std::to_string(real_pos[1]) + ", " + 
                       std::to_string(real_yaw) + ")";

        {
            std::lock_guard<std::mutex> lk(user_mtx_);
            inner["user"] = "(" + std::to_string(user_pose_[0]) + ", " + 
                            std::to_string(user_pose_[1]) + ", " + 
                            std::to_string(user_pose_[2]) + ")";
        }

        {
            std::lock_guard<std::mutex> lk(particles_mtx_);
            if (particles_.empty()) {
                inner["particles"] = "[]";
            } else {
                inner["particles"] = json(particles_).dump();
            }
        }

        return inner;
    }

    void on_frontend_message(const std::string& msg) override {
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

        std::vector<uchar> buf;
        cv::imencode(".jpg", local, buf, { cv::IMWRITE_JPEG_QUALITY, 60 });

        json p;
        p["image"] = base64_encode(buf.data(), buf.size());
        p["shape"] = std::vector<int>{ local.rows, local.cols, local.channels() };

        last_image_payload_ = p.dump();
        return last_image_payload_;
    }

    std::shared_ptr<OdometryNode> odom_node_;
    rclcpp::Node::SharedPtr aux_node_;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr estimated_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr particles_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr debug_sub_;

    cv::Mat img_buf_;
    std::mutex img_mtx_;
    std::atomic<bool> image_updated_;
    std::string last_image_payload_;

    std::array<double, 3> user_pose_;
    std::mutex user_mtx_;

    std::vector<std::vector<double>> particles_;
    std::mutex particles_mtx_;

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

void WebGUI::set_image(const cv::Mat& image)
{
    if (gui_node_) gui_node_->set_image(image);
}

void WebGUI::show_position(double x, double y, double angle)
{
    if (gui_node_) gui_node_->show_position(x, y, angle);
}

void WebGUI::show_particles(const std::vector<std::vector<double>>& particles)
{
    if (gui_node_) gui_node_->show_particles(particles);
}

std::vector<double> WebGUI::pose_to_map(double x_prime, double y_prime, double yaw_prime)
{
    double x = 101.1 * (4.2 + y_prime);
    double y = 101.1 * (5.7 - x_prime);
    double yaw = yaw_prime - M_PI / 2.0;
    return {std::round(x), std::round(y), yaw};
}

std::vector<double> WebGUI::map_to_pose(double map_x, double map_y, double map_yaw)
{
    double x = (map_y - 576.27) / -101.1;
    double y = (map_x - 424.62) / 101.1;
    double yaw = map_yaw + M_PI / 2.0;
    return {x, y, yaw};
}

cv::Mat WebGUI::get_map(const std::string& url)
{
    if (gui_node_) return gui_node_->get_map(url);
    return cv::Mat();
}

cv::Mat WebGUI::get_bgr_map(const std::string& url)
{
    if (gui_node_) return gui_node_->get_bgr_map(url);
    return cv::Mat();
}