#include "WebGUI.hpp"
#include "Map.hpp"
#include "Lap.hpp"
#include "common_interfaces_cpp/webgui/WebGUIBridge.hpp"
#include "common_interfaces_cpp/webgui/RTFMonitor.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"
#include "common_interfaces_cpp/hal/laser.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"
#include <atomic>

using json = nlohmann::json;

class WebGUINode : public BaseWebGUI
{
public:
    WebGUINode()
        : BaseWebGUI("webgui_node", "127.0.0.1", "2303", 30.0)
        , gui_iterations_(0)
        , rtf_monitor_("/stats", std::chrono::milliseconds(500))
    {
        odom_node_ = std::make_shared<OdometryNode>("/odom", "webgui_odom");
        laser_node_ = std::make_shared<LaserNode>("/f1/laser/scan", "webgui_laser");
        lap_ = std::make_shared<Lap>(odom_node_);
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
        return { shared_from_this(), odom_node_, laser_node_, aux_node_ };
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
        inner["lap"] = lap_->check_threshold();
        inner["map"] = map_->get_json_data().dump();
        return inner;
    }

    void on_frontend_message(const std::string& msg) override {
        if (msg.find("startLap") != std::string::npos) lap_->unpause();
        else if (msg.find("pause") != std::string::npos) lap_->pause();
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
    std::shared_ptr<Lap> lap_;
    std::shared_ptr<Map> map_;
    
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