#include "WebGUI.hpp"
#include "common_interfaces_cpp/webgui/WebGUIBridge.hpp"
#include "common_interfaces_cpp/webgui/RTFMonitor.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"
#include <atomic>

class WebGUINode : public BaseWebGUI
{
public:
    WebGUINode()
        : BaseWebGUI("webgui_node", "127.0.0.1", "2303", 20.0)
        , rtf_monitor_("/stats", std::chrono::milliseconds(500))
        , gui_iterations_(0)
    {
        odom_node_ = std::make_shared<OdometryNode>("/vacuum_cleaner/odom", "webgui_odom");
        last_stat_time_ = std::chrono::steady_clock::now();
        stats_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&WebGUINode::send_stats, this));
    }

    std::vector<rclcpp::Node::SharedPtr> get_internal_nodes() {
        return { shared_from_this(), odom_node_ };
    }

protected:
    json update_gui() override {
        gui_iterations_++;
        Pose3d pose = odom_node_->getPose3d();
        const double x = -30.0 * pose.x + 171.0;
        const double y = 15.0 * pose.y + 63.0;
        json inner;
        inner["map"] = json{x, y, pose.yaw}.dump();
        return inner;
    }

    void on_frontend_message(const std::string& /*msg*/) override {}

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
        stats["gui"]   = 20.0;
        stats["rtf"]   = rtf_monitor_.get();
        stats["fps"]   = -1.0;
        stats["lat"]   = -1.0;
        send_to_frontend(stats);
    }

    std::shared_ptr<OdometryNode> odom_node_;
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
