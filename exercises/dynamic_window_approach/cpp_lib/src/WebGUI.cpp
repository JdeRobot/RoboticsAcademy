#include "WebGUI.hpp"
#include <thread>
#include <chrono>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

std::shared_ptr<Map> WebGUINode::map_ = nullptr;
std::shared_ptr<Lap> WebGUINode::lap_ = nullptr;
std::shared_ptr<OdometryNode> WebGUINode::pose3d_node_ = nullptr;
std::shared_ptr<LaserNode> WebGUINode::laser_node_ = nullptr;

WebGUINode::WebGUINode() : Node("webgui_bridge_node") {
    if (!pose3d_node_) pose3d_node_ = std::make_shared<OdometryNode>("/odom", "webgui_odom_node");
    if (!laser_node_) laser_node_ = std::make_shared<LaserNode>("/f1/laser/scan");

    if (!map_) {
        map_ = std::make_shared<Map>(
            []() { 
                LaserData d = WebGUINode::laser_node_->getLaserData(); 
                while (d.values.empty() && rclcpp::ok()) { 
                    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Evita que la CPU se congele esperando
                    d = WebGUINode::laser_node_->getLaserData(); 
                }
                return d; 
            },
            []() { return WebGUINode::pose3d_node_->getPose3d(); }
        );
        lap_ = std::make_shared<Lap>(map_);

        std::thread spin_thread([]() {
            rclcpp::executors::SingleThreadedExecutor executor;
            executor.add_node(WebGUINode::pose3d_node_);
            executor.add_node(WebGUINode::laser_node_);
            while (rclcpp::ok()) {
                executor.spin_some();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
        spin_thread.detach();
    }

    auto cb_car = [](geometry_msgs::msg::Point::UniquePtr m) { map_->setCar(m->x, m->y); };
    auto cb_obs = [](geometry_msgs::msg::Point::UniquePtr m) { map_->setObs(m->x, m->y); };
    auto cb_avg = [](geometry_msgs::msg::Point::UniquePtr m) { map_->setAvg(m->x, m->y); };
    auto cb_target = [](geometry_msgs::msg::Point::UniquePtr m) { map_->setTargetPos(m->x, m->y); };

    sub_car_ = create_subscription<geometry_msgs::msg::Point>("/webgui/force/car", 10, cb_car);
    sub_obs_ = create_subscription<geometry_msgs::msg::Point>("/webgui/force/obs", 10, cb_obs);
    sub_avg_ = create_subscription<geometry_msgs::msg::Point>("/webgui/force/avg", 10, cb_avg);
    sub_target_ = create_subscription<geometry_msgs::msg::Point>("/webgui/local_target", 10, cb_target);
    
    sub_reached_ = create_subscription<std_msgs::msg::Bool>("/webgui/target_reached", 10, 
        std::bind(&WebGUINode::target_reached_callback, this, _1));

    rclcpp::QoS qos(1); qos.transient_local();
    pub_current_target_ = create_publisher<geometry_msgs::msg::Point>("/webgui/current_target", qos);

    current_target_obj_ = map_->getNextTarget();
    publish_current_target();
}

void WebGUINode::target_reached_callback(std_msgs::msg::Bool::UniquePtr msg) {
    if (msg->data && current_target_obj_ && map_) {
        current_target_obj_->setReached(true);
        current_target_obj_ = map_->getNextTarget();
        publish_current_target();
    }
}

void WebGUINode::publish_current_target() {
    if (current_target_obj_) {
        geometry_msgs::msg::Point m;
        m.x = current_target_obj_->getPose().x;
        m.y = current_target_obj_->getPose().y;
        m.z = 0.0;
        pub_current_target_->publish(m);
    }
}

// WebSocket Session Helper
class session : public std::enable_shared_from_this<session> {
    tcp::resolver resolver_;
    websocket::stream<beast::tcp_stream> ws_;
    beast::flat_buffer buffer_;
    std::string host_, text_;

public:
    explicit session(net::io_context& ioc) : resolver_(net::make_strand(ioc)), ws_(net::make_strand(ioc)) {}

    void run(char const* host, char const* port, char const* text) {
        host_ = host; text_ = text;
        buffer_.max_size(1024 * 1024);
        resolver_.async_resolve(host, port, beast::bind_front_handler(&session::on_resolve, shared_from_this()));
    }

    void on_resolve(beast::error_code ec, tcp::resolver::results_type results) {
        if (ec) return;
        beast::get_lowest_layer(ws_).expires_after(std::chrono::seconds(30));
        beast::get_lowest_layer(ws_).async_connect(results, beast::bind_front_handler(&session::on_connect, shared_from_this()));
    }

    void on_connect(beast::error_code ec, tcp::resolver::results_type::endpoint_type ep) {
        if (ec) return;
        beast::get_lowest_layer(ws_).expires_never();
        ws_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::client));
        ws_.read_message_max(1024 * 1024);
        ws_.auto_fragment(false);
        host_ += ':' + std::to_string(ep.port());
        ws_.async_handshake(host_, "", beast::bind_front_handler(&session::on_handshake, shared_from_this()));
    }

    void on_handshake(beast::error_code ec) {
        if (ec) return;
        ws_.async_write(net::buffer(text_), beast::bind_front_handler(&session::on_write, shared_from_this()));
    }

    void on_write(beast::error_code ec, std::size_t) {
        if (ec) return;
        ws_.async_read(buffer_, beast::bind_front_handler(&session::on_read, shared_from_this()));
    }

    void on_read(beast::error_code ec, std::size_t) {
        if (ec) return;
        buffer_.consume(buffer_.size());

        json j;
        j["lap"] = "";
        j["map"] = "";

        if (WebGUINode::lap_) {
            auto lapped = WebGUINode::lap_->check_threshold();
            j["lap"] = std::to_string(lapped);
        }
        
        if (WebGUINode::map_) {
            j["map"] = WebGUINode::map_->get_json_data();
        }

        std::string out = j.dump();
        ws_.async_write(net::buffer(out), beast::bind_front_handler(&session::on_write, shared_from_this()));
    }
};

WebGUI::WebGUI() {
    net::io_context ioc;
    std::make_shared<session>(ioc)->run("127.0.0.1", "2303", "{\"lap\":\"\",\"map\":\"\"}");
    ioc.run();
}

void WebGUI::showForces(const std::vector<double>& v1, const std::vector<double>& v2, const std::vector<double>& v3) {
    if (WebGUINode::map_) {
        WebGUINode::map_->setCar(v1[0], v1[1]);
        WebGUINode::map_->setObs(v2[0], v2[1]);
        WebGUINode::map_->setAvg(v3[0], v3[1]);
    }
}

void WebGUI::showLocalTarget(const std::vector<double>& v) {
    if (WebGUINode::map_) WebGUINode::map_->setTargetPos(v[0], v[1]);
}

std::shared_ptr<Target> WebGUI::getNextTarget() {
    return WebGUINode::map_ ? WebGUINode::map_->getNextTarget() : nullptr;
}

void WebGUI::setTargetx(double x) {
    if (WebGUINode::map_) WebGUINode::map_->targetx = x;
}

void WebGUI::setTargety(double y) {
    if (WebGUINode::map_) WebGUINode::map_->targety = y;
}