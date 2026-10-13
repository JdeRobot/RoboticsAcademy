#include "WebGUI.hpp"
#include "Map.hpp"
#include "common_interfaces_cpp/webgui/WebGUIBridge.hpp"
#include "common_interfaces_cpp/webgui/RTFMonitor.hpp"
#include "common_interfaces_cpp/hal/laser.hpp"
#include "common_interfaces_cpp/hal/lidar.hpp"
#include <nlohmann/json.hpp>
#include <atomic>
#include <cmath>
#include <vector>

using json = nlohmann::json;

class WebGUINode : public BaseWebGUI
{
public:
    WebGUINode()
        : BaseWebGUI("autoparking_gui_node", "127.0.0.1", "2303", 30.0)
        , gui_iterations_(0)
        , rtf_monitor_("/stats", std::chrono::milliseconds(500))
        , laser_front_node_(std::make_shared<LaserNode>("/autonomous_car/laser_front/scan", "webgui_laser_front"))
        , laser_right_node_(std::make_shared<LaserNode>("/autonomous_car/laser_side/scan", "webgui_laser_right"))
        , laser_back_node_(std::make_shared<LaserNode>("/autonomous_car/laser_back/scan", "webgui_laser_back"))
        , lidar_node_(std::make_shared<LidarNode>("/autonomous_car/lidar/pc2/points", "webgui_lidar"))
        , aux_node_(std::make_shared<rclcpp::Node>("webgui_aux"))
        , map_util_([this](){ return laser_front_node_->getLaserData(); },
                    [this](){ return laser_right_node_->getLaserData(); },
                    [this](){ return laser_back_node_->getLaserData(); })
    {
        last_stat_time_ = std::chrono::steady_clock::now();
        stats_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&WebGUINode::send_stats, this));
    }

    std::vector<rclcpp::Node::SharedPtr> get_internal_nodes() {
        return { shared_from_this(), laser_front_node_, laser_right_node_, laser_back_node_, lidar_node_, aux_node_ };
    }

protected:
    json update_gui() override {
        gui_iterations_++;
        json inner;

        // Detect mode elegantly using ROS 2 native publisher count instead of a subprocess
        std::string mode = "Laser";
        if (aux_node_->count_publishers("/autonomous_car/lidar/pc2/points") > 0) {
            mode = "Lidar";
        }

        if (mode == "Laser") {
            auto lf = laser_front_node_->getLaserData();
            auto lr = laser_right_node_->getLaserData();
            auto lb = laser_back_node_->getLaserData();

            if (!lf.values.empty() && !lr.values.empty() && !lb.values.empty()) {
                // map_util_.get_json_data() returns a string dump of the payload
                inner["map"] = map_util_.get_json_data();
            }
        } 
        else if (mode == "Lidar") {
            auto lidar = lidar_node_->getLidarData();
            if (!lidar.points.empty()) {
                std::vector<std::vector<double>> points_to_paint;
                
                for (const auto& p : lidar.points) {
                    double x = p[0];
                    double y = p[1];
                    double z = p[2];
                    
                    if (!std::isinf(x) && !std::isinf(y) && !std::isinf(z) &&
                        !std::isnan(x) && !std::isnan(y) && !std::isnan(z)) {
                        
                        // Color format: x, y, z, r, g, b (Color: 20, 20, 255)
                        points_to_paint.push_back({
                            x * 10.0, 
                            (z + 1.75) * 10.0, 
                            -y * 10.0, 
                            20.0, 
                            20.0, 
                            255.0
                        });
                    }
                }
                inner["lidar"] = json(points_to_paint).dump();
            }
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
        stats["gui"]   = 30.0;
        stats["rtf"]   = rtf_monitor_.get();
        stats["fps"]   = -1.0;
        stats["lat"]   = -1.0;

        send_to_frontend(stats);
    }

    std::shared_ptr<LaserNode> laser_front_node_;
    std::shared_ptr<LaserNode> laser_right_node_;
    std::shared_ptr<LaserNode> laser_back_node_;
    std::shared_ptr<LidarNode> lidar_node_;
    rclcpp::Node::SharedPtr aux_node_;
    
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