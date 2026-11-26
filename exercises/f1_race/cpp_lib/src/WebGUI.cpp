#include "WebGUI.hpp"
#include "Map.hpp"
#include "common_interfaces_cpp/webgui/WebGUIBridge.hpp"
#include "common_interfaces_cpp/webgui/RTFMonitor.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"
#include "common_interfaces_cpp/hal/laser.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include <atomic>
#include <mutex>

using json = nlohmann::json;

class WebGUINode : public BaseWebGUI
{
public:
    WebGUINode()
        : BaseWebGUI("webgui_node", "127.0.0.1", "2303", 30.0)
        , gui_iterations_(0)
        , rtf_monitor_("/stats", std::chrono::milliseconds(500))
    {
        odom_node_ = std::make_shared<OdometryNode>("/f1/odom", "webgui_odom");
        laser_node_ = std::make_shared<LaserNode>("/f1/laser/scan", "webgui_laser");
        rival_odom_node_ = std::make_shared<OdometryNode>("/f1_rival/odom", "webgui_rival_odom");
        map_ = std::make_shared<Map>(laser_node_, odom_node_);
        aux_node_ = std::make_shared<rclcpp::Node>("webgui_aux");

        setup_ros2_communications();

        last_stat_time_ = std::chrono::steady_clock::now();
        stats_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&WebGUINode::send_stats, this));

        publish_current_target();
    }

    std::vector<rclcpp::Node::SharedPtr> get_internal_nodes() {
        return { shared_from_this(), odom_node_, laser_node_, rival_odom_node_, aux_node_ };
    }

    void show_forces(const std::array<double, 2>& vec1, const std::array<double, 2>& vec2, const std::array<double, 2>& vec3) {
        map_->set_car(vec1[0], vec1[1]);
        map_->set_obs(vec2[0], vec2[1]);
        map_->set_avg(vec3[0], vec3[1]);
    }

    void show_local_target(const std::array<double, 2>& new_vec) {
        map_->set_target_pos(new_vec[0], new_vec[1]);
    }

    std::array<double, 2> get_next_target() {
        return map_->get_next_target();
    }

    void set_target_x(double x) {
        map_->set_target_x(x);
    }

    void set_target_y(double y) {
        map_->set_target_y(y);
    }

    void mark_target_reached() {
        map_->mark_current_target_reached();
        publish_current_target();
    }

protected:
    json update_gui() override {
        gui_iterations_++;
        json inner;
        inner["map"] = map_->get_json_data().dump();

        // Both cars for the aerial map, yaw goes along so it can draw where each points
        Pose3d pose = odom_node_->getPose3d();
        inner["car"] = "(" + std::to_string(pose.x) + ", " + std::to_string(pose.y) +
                       ", " + std::to_string(pose.yaw) + ")";
        Pose3d rival = rival_odom_node_->getPose3d();
        inner["rival"] = "(" + std::to_string(rival.x) + ", " +
                         std::to_string(rival.y) + ", " +
                         std::to_string(rival.yaw) + ")";
        {
            std::lock_guard<std::mutex> lk(status_mtx_);
            inner["race"] = status_;
        }
        return inner;
    }

private:
    void setup_ros2_communications() {
        force_car_sub_ = aux_node_->create_subscription<geometry_msgs::msg::Point>(
            "/webgui/force/car", 10, [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                map_->set_car(msg->x, msg->y);
            });

        force_obs_sub_ = aux_node_->create_subscription<geometry_msgs::msg::Point>(
            "/webgui/force/obs", 10, [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                map_->set_obs(msg->x, msg->y);
            });

        force_avg_sub_ = aux_node_->create_subscription<geometry_msgs::msg::Point>(
            "/webgui/force/avg", 10, [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                map_->set_avg(msg->x, msg->y);
            });

        target_sub_ = aux_node_->create_subscription<geometry_msgs::msg::Point>(
            "/webgui/local_target", 10, [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                map_->set_target_pos(msg->x, msg->y);
            });

        target_reached_sub_ = aux_node_->create_subscription<std_msgs::msg::Bool>(
            "/webgui/target_reached", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) {
                    mark_target_reached();
                }
            });

        // Race status latched by the rival
        auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        status_sub_ = aux_node_->create_subscription<std_msgs::msg::String>(
            "/f1_race/status", latched, [this](const std_msgs::msg::String::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(status_mtx_);
                status_ = msg->data;
            });

        auto qos_target = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        target_pub_ = aux_node_->create_publisher<geometry_msgs::msg::Point>(
            "/webgui/current_target", qos_target);
    }

    void publish_current_target() {
        auto coords = map_->get_next_target();
        geometry_msgs::msg::Point msg;
        msg.x = coords[0];
        msg.y = coords[1];
        msg.z = 0.0;
        target_pub_->publish(msg);
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

    std::shared_ptr<OdometryNode> odom_node_;
    std::shared_ptr<LaserNode> laser_node_;
    std::shared_ptr<OdometryNode> rival_odom_node_;
    std::shared_ptr<Map> map_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    std::mutex status_mtx_;
    std::string status_;
    
    rclcpp::Node::SharedPtr aux_node_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr force_car_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr force_obs_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr force_avg_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_reached_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pub_;

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

void WebGUI::show_forces(const std::array<double, 2>& vec1, const std::array<double, 2>& vec2, const std::array<double, 2>& vec3)
{
    if (gui_node_) gui_node_->show_forces(vec1, vec2, vec3);
}

void WebGUI::show_local_target(const std::array<double, 2>& new_vec)
{
    if (gui_node_) gui_node_->show_local_target(new_vec);
}

std::array<double, 2> WebGUI::get_next_target()
{
    if (gui_node_) return gui_node_->get_next_target();
    return {0.0, 0.0};
}

void WebGUI::set_target_x(double x)
{
    if (gui_node_) gui_node_->set_target_x(x);
}

void WebGUI::set_target_y(double y)
{
    if (gui_node_) gui_node_->set_target_y(y);
}

void WebGUI::mark_target_reached()
{
    if (gui_node_) gui_node_->mark_target_reached();
}