#include "WebGUI.hpp"
#include "Map.hpp"
#include "common_interfaces_cpp/webgui/WebGUIBridge.hpp"
#include "common_interfaces_cpp/webgui/RTFMonitor.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <atomic>
#include <algorithm>
#include <string>
#include <sstream>

class WebGUINode : public BaseWebGUI
{
public:
    WebGUINode()
        : BaseWebGUI("webgui_node", "127.0.0.1", "2303", 30.0)
        , image_updated_(false)
        , last_image_payload_("{\"image\":\"\",\"shape\":[]}")
        , gui_iterations_(0)
        , rtf_monitor_("/stats", std::chrono::milliseconds(500))
        , map_util_([this](){ return odom_node_->getPose3d(); })
    {
        odom_node_ = std::make_shared<OdometryNode>("/odom", "webgui_odom");
        aux_node_ = std::make_shared<rclcpp::Node>("webgui_aux");

        auto qos_transient = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        target_pub_ = aux_node_->create_publisher<geometry_msgs::msg::Point>(
            "/webgui/current_target", qos_transient);

        auto qos_volatile = rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().best_effort();
        
        debug_sub_ = aux_node_->create_subscription<sensor_msgs::msg::Image>(
            "/webgui/debug_image", qos_volatile,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                try {
                    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
                    show_image(img);
                } catch (...) {}
            });

        path_sub_ = aux_node_->create_subscription<nav_msgs::msg::Path>(
            "/webgui/path", qos_volatile,
            [this](const nav_msgs::msg::Path::SharedPtr msg) {
                std::vector<std::vector<int>> path;
                for (const auto& pose : msg->poses) {
                    path.push_back(map_util_.worldToGrid(pose.pose.position.x, pose.pose.position.y));
                }
                show_path(path);
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
        std::lock_guard<std::mutex> lk(img_mtx_);
        img.copyTo(img_buf_);
        image_updated_.store(true);
    }

    void show_path(const std::vector<std::vector<int>>& path) {
        std::lock_guard<std::mutex> lk(path_mtx_);
        current_path_ = path;
    }

    std::vector<double> get_target_pose() {
        std::lock_guard<std::mutex> lk(target_mtx_);
        return current_target_;
    }

    cv::Mat get_map(const std::string& url) {
        return map_util_.getMap(url);
    }

    std::vector<int> world_to_grid(const std::vector<double>& pose) {
        if (pose.size() < 2) return {0, 0};
        return map_util_.worldToGrid(pose[0], pose[1]);
    }

    std::vector<double> grid_to_world(const std::vector<double>& cell) {
        if (cell.size() < 2) return {0.0, 0.0};
        return map_util_.gridToWorld(static_cast<int>(cell[0]), static_cast<int>(cell[1]));
    }

protected:
    json update_gui() override {
        gui_iterations_++;

        json inner;
        inner["image"] = encode_image();
        
        std::string path_str = "[]";
        {
            std::lock_guard<std::mutex> lk(path_mtx_);
            path_str = json(current_path_).dump();
        }
        inner["array"] = path_str;

        Pose3d pose = odom_node_->getPose3d();
        std::vector<int> grid_pos = map_util_.worldToGrid(pose.x, pose.y);
        
        inner["map"] = "(" + std::to_string(grid_pos[0]) + ", " + 
                       std::to_string(grid_pos[1]) + ", " + 
                       std::to_string(pose.yaw) + ")";

        return inner;
    }

    void on_frontend_message(const std::string& msg) override {
        if (msg.find("pick") == 0) {
            std::string coords_str = msg.substr(4);
            std::replace(coords_str.begin(), coords_str.end(), ',', ' ');

            std::istringstream iss(coords_str);
            double grid_x, grid_y;
            
            if (iss >> grid_x >> grid_y) {
                std::vector<double> world_xy = map_util_.gridToWorld(static_cast<int>(grid_x), static_cast<int>(grid_y));

                {
                    std::lock_guard<std::mutex> lk(target_mtx_);
                    current_target_ = world_xy;
                }

                geometry_msgs::msg::Point pt;
                pt.x = world_xy[0];
                pt.y = world_xy[1];
                pt.z = 0.0;
                target_pub_->publish(pt);
            }
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
        p["shape"] = std::vector<int>{ out.rows, out.cols, 3 };

        last_image_payload_ = p.dump();
        return last_image_payload_;
    }

    std::shared_ptr<OdometryNode> odom_node_;
    rclcpp::Node::SharedPtr aux_node_;
    
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr debug_sub_;

    cv::Mat img_buf_;
    std::mutex img_mtx_;
    std::atomic<bool> image_updated_;
    std::string last_image_payload_;

    std::vector<std::vector<int>> current_path_;
    std::mutex path_mtx_;

    std::vector<double> current_target_;
    std::mutex target_mtx_;

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

void WebGUI::show_path(const std::vector<std::vector<int>>& path)
{
    if (gui_node_) gui_node_->show_path(path);
}

std::vector<double> WebGUI::get_target_pose()
{
    if (gui_node_) return gui_node_->get_target_pose();
    return {};
}

cv::Mat WebGUI::get_map(const std::string& url)
{
    if (gui_node_) return gui_node_->get_map(url);
    return cv::Mat();
}

std::vector<int> WebGUI::world_to_grid(const std::vector<double>& pose)
{
    if (gui_node_) return gui_node_->world_to_grid(pose);
    return {0, 0};
}

std::vector<double> WebGUI::grid_to_world(const std::vector<double>& cell)
{
    if (gui_node_) return gui_node_->grid_to_world(cell);
    return {0.0, 0.0};
}