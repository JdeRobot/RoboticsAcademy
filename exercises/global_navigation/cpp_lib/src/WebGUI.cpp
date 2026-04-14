#include "WebGUI.hpp"
#include <cv_bridge/cv_bridge.h>
#include <boost/beast/core/detail/base64.hpp>
#include <regex>

using namespace std::chrono_literals;
using std::placeholders::_1;

std::shared_ptr<Map> WebGUINode::map_ = nullptr;
std::shared_ptr<OdometryNode> WebGUINode::pose3d_node_ = nullptr;
std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Point>> WebGUINode::target_pub_ = nullptr;

std::vector<double> WebGUI::worldXY = {};
std::string WebGUI::array_str = "[]";
cv::Mat WebGUI::image_to_be_shown = cv::Mat();
bool WebGUI::image_to_be_shown_updated = false;
std::mutex WebGUI::image_show_lock;
std::mutex WebGUI::array_lock;

WebGUINode::WebGUINode() : Node("gui_bridge_node")
{
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local();

    target_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/webgui/current_target", qos);

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/webgui/path", 10, std::bind(&WebGUINode::path_callback, this, _1));
    
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/webgui/debug_image", 10, std::bind(&WebGUINode::image_callback, this, _1));
}

void WebGUINode::publish_target(double x, double y)
{
    if (target_pub_) {
        geometry_msgs::msg::Point msg;
        msg.x = x;
        msg.y = y;
        msg.z = 0.0;
        target_pub_->publish(msg);
    }
}

void WebGUINode::path_callback(nav_msgs::msg::Path::UniquePtr msg)
{
    std::vector<std::vector<int>> path_array;
    if (WebGUINode::map_) {
        for (const auto& pose_stamped : msg->poses) {
            auto coords = WebGUINode::map_->worldToGrid(pose_stamped.pose.position.x, pose_stamped.pose.position.y);
            path_array.push_back(coords);
        }
    }
    WebGUI::showPath(path_array);
}

void WebGUINode::image_callback(sensor_msgs::msg::Image::UniquePtr msg)
{
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::MONO8);
        WebGUI::showNumpy(cv_ptr->image);
    } catch (...) {}
}

void WebGUI::showNumpy(const cv::Mat& image)
{
    cv::Mat processed;
    if (image.channels() == 1) {
        cv::cvtColor(image, processed, cv::COLOR_GRAY2BGR);
    } else {
        processed = image.clone();
    }

    std::lock_guard<std::mutex> lock(image_show_lock);
    image_to_be_shown = processed;
    image_to_be_shown_updated = true;
}

void WebGUI::showPath(const std::vector<std::vector<int>>& array)
{
    std::lock_guard<std::mutex> lock(array_lock);
    json j = array;
    array_str = j.dump();
}

std::vector<double> WebGUI::getTargetPose() { return worldXY; }

cv::Mat WebGUI::getMap(const std::string& url) 
{ 
    if (WebGUINode::map_) return WebGUINode::map_->getMap(url); 
    return cv::Mat(); 
}

std::vector<int> WebGUI::rowColumn(const std::vector<double>& pose) 
{ 
    if (WebGUINode::map_) return WebGUINode::map_->rowColumn(pose); 
    return {}; 
}

std::vector<int> WebGUI::worldToGrid(const std::vector<double>& pose) 
{ 
    if (WebGUINode::map_ && pose.size() >= 2) return WebGUINode::map_->worldToGrid(pose[0], pose[1]); 
    return {}; 
}

std::vector<double> WebGUI::gridToWorld(const std::vector<int>& cell) 
{ 
    if (WebGUINode::map_ && cell.size() >= 2) return WebGUINode::map_->gridToWorld(cell[0], cell[1]); 
    return {}; 
}

void WebGUI::reset_gui() 
{ 
    cv::Mat empty_img = cv::Mat::zeros(400, 400, CV_8UC3);
    showNumpy(empty_img);
    if (WebGUINode::map_) WebGUINode::map_->reset(); 
}

std::string WebGUI::payloadImage()
{
    std::lock_guard<std::mutex> lock(image_show_lock);
    json payload = {{"image", ""}, {"shape", {}}};
    
    if (!image_to_be_shown_updated || image_to_be_shown.empty()) {
        return payload.dump();
    }

    std::vector<uchar> buf;
    cv::imencode(".JPEG", image_to_be_shown, buf);
    std::string encoded_image;
    encoded_image.resize(boost::beast::detail::base64::encoded_size(buf.size()));
    boost::beast::detail::base64::encode(&encoded_image[0], buf.data(), buf.size());

    payload["image"] = encoded_image;
    payload["shape"] = {image_to_be_shown.rows, image_to_be_shown.cols, image_to_be_shown.channels()};
    
    image_to_be_shown_updated = false;
    return payload.dump();
}

class session : public std::enable_shared_from_this<session>
{
    tcp::resolver resolver_;
    websocket::stream<beast::tcp_stream> ws_;
    beast::flat_buffer buffer_;
    std::string host_;

public:
    explicit session(net::io_context &ioc) : resolver_(net::make_strand(ioc)), ws_(net::make_strand(ioc)) {}

    void run(char const *host, char const *port) {
        host_ = host;
        buffer_.max_size(1024 * 1024 * 10);
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
        ws_.read_message_max(1024 * 1024 * 10);
        ws_.auto_fragment(false);
        host_ += ':' + std::to_string(ep.port());
        ws_.async_handshake(host_, "/", beast::bind_front_handler(&session::on_handshake, shared_from_this()));
    }

    void on_handshake(beast::error_code ec) {
        if (ec) return;
        ws_.async_read(buffer_, beast::bind_front_handler(&session::on_read, shared_from_this()));
    }

    void on_read(beast::error_code ec, std::size_t) {
        if (ec) return;
        std::string msg = beast::buffers_to_string(buffer_.data());
        buffer_.consume(buffer_.size());

        if (msg.find("pick") != std::string::npos) {
            std::regex re("[-+]?[0-9]*\\.?[0-9]+");
            auto words_begin = std::sregex_iterator(msg.begin(), msg.end(), re);
            auto words_end = std::sregex_iterator();

            std::vector<double> coords;
            for (std::sregex_iterator i = words_begin; i != words_end && coords.size() < 2; ++i) {
                coords.push_back(std::stod(i->str()));
            }

            if (coords.size() == 2 && WebGUINode::map_) {
                WebGUI::worldXY = WebGUINode::map_->gridToWorld(static_cast<int>(coords[0]), static_cast<int>(coords[1]));
                if (!WebGUI::worldXY.empty()) {
                    WebGUINode::publish_target(WebGUI::worldXY[0], WebGUI::worldXY[1]);
                }
            }
        }

        std::string map_pos = "";
        if (WebGUINode::map_) {
            auto coords = WebGUINode::map_->getTaxiCoordinates();
            auto angle = WebGUINode::map_->getTaxiAngle();
            if (!coords.empty() && !angle.empty()) {
                map_pos = "(" + std::to_string(coords[0]) + ", " + std::to_string(coords[1]) + ", " + std::to_string(angle[0]) + ")";
            }
        }

        json payload = {
            {"image", WebGUI::payloadImage()},
            {"array", WebGUI::array_str},
            {"map", map_pos}
        };

        std::string out = payload.dump();
        ws_.async_write(net::buffer(out), beast::bind_front_handler(&session::on_write, shared_from_this()));
    }

    void on_write(beast::error_code ec, std::size_t) {
        if (ec) return;
        ws_.async_read(buffer_, beast::bind_front_handler(&session::on_read, shared_from_this()));
    }
};

WebGUI::WebGUI()
{
    if (!WebGUINode::pose3d_node_) {
        WebGUINode::pose3d_node_ = std::make_shared<OdometryNode>("/odom", "webgui_odom_node");
        
        // Hilo independiente para hacer el spin del nodo de odometría del GUI
        std::thread([]() {
            rclcpp::executors::SingleThreadedExecutor exec;
            exec.add_node(WebGUINode::pose3d_node_);
            exec.spin();
        }).detach();
    }

    if (!WebGUINode::map_) {
        WebGUINode::map_ = std::make_shared<Map>([]() { return WebGUINode::pose3d_node_->getPose3d(); });
    }
    
    net::io_context ioc;
    std::make_shared<session>(ioc)->run("127.0.0.1", "2303");
    ioc.run();
}