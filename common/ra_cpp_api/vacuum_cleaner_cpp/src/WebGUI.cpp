#include "vacuum_cleaner_cpp/WebGUI.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;
using std::placeholders::_1;

// Statics
std::string WebGUI::img_payload = "";
nav_msgs::msg::Odometry WebGUINode::last_odom = nav_msgs::msg::Odometry();
gazebo_msgs::msg::PerformanceMetrics WebGUINode::last_perf = gazebo_msgs::msg::PerformanceMetrics();

// WebGUINode Implementation
WebGUINode::WebGUINode() : Node("webgui_node")
{
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&WebGUINode::pose_callback, this, _1));
    perf_sub_ = create_subscription<gazebo_msgs::msg::PerformanceMetrics>(
        "/performance_metrics", 10, std::bind(&WebGUINode::performance_callback, this, _1));
}

std::vector<double> WebGUINode::get_pose()
{
    tf2::Quaternion tf_quat;
    tf2::fromMsg(last_odom.pose.pose.orientation, tf_quat);
    tf2::Matrix3x3 m(tf_quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    const double x = -30.0 * last_odom.pose.pose.position.x + 171.0;
    const double y = 15.0 * last_odom.pose.pose.position.y + 63.0;
    return {x, y, yaw};
}

double WebGUINode::get_performance() { return last_perf.real_time_factor; }

void WebGUINode::pose_callback(nav_msgs::msg::Odometry::UniquePtr msg) { last_odom = *msg; }
void WebGUINode::performance_callback(gazebo_msgs::msg::PerformanceMetrics::UniquePtr msg) { last_perf = *msg; }

// WebSocket Session Helper
class session : public std::enable_shared_from_this<session>
{
    tcp::resolver resolver_;
    websocket::stream<beast::tcp_stream> ws_;
    beast::flat_buffer buffer_;
    std::string host_, text_;

public:
    explicit session(net::io_context &ioc) : resolver_(net::make_strand(ioc)), ws_(net::make_strand(ioc)) {}

    void run(char const *host, char const *port, char const *text) {
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
        unsigned char *cp = (unsigned char *)buffer_.data().data();
        std::string msg(reinterpret_cast<char const *>(cp));
        buffer_.consume(buffer_.size());

        auto pose = WebGUINode::get_pose();
        json map_j = {pose.at(0), pose.at(1), pose.at(2)};
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << WebGUINode::get_performance();
        
        json j = {{"map", map_j.dump()}, {"brain", Frequency::rate}, {"gui", 20}, {"rtf", ss.str()}, {"fps", -1}, {"lat", -1}};
        std::string out = j.dump();
        ws_.async_write(net::buffer(out), beast::bind_front_handler(&session::on_write, shared_from_this()));
    }
};

WebGUI::WebGUI()
{
    net::io_context ioc;
    std::make_shared<session>(ioc)->run("127.0.0.1", "2303", "{\"map\":\"(201,85.5,0)\"}");
    ioc.run();
}