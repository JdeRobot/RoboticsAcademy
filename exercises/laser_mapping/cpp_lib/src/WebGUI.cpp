#include "WebGUI.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <boost/asio/steady_timer.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

Pose3D WebGUINode::last_pose_ = Pose3D();
Pose3D WebGUINode::last_noisy_pose_ = Pose3D();
cv::Mat WebGUINode::user_map_ = cv::Mat();
std::mutex WebGUINode::map_mutex_;

WebGUINode::WebGUINode() : Node("gui_bridge_node")
{
    user_map_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/webgui/user_map", rclcpp::QoS(1).transient_local(), std::bind(&WebGUINode::user_map_callback, this, _1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/turtlebot3/odom", rclcpp::SensorDataQoS(), std::bind(&WebGUINode::odom_callback, this, _1));

    noisy_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/turtlebot3/odom_noisy", rclcpp::SensorDataQoS(), std::bind(&WebGUINode::noisy_odom_callback, this, _1));
}

Pose3D WebGUINode::get_pose() { return last_pose_; }
Pose3D WebGUINode::get_noisy_pose() { return last_noisy_pose_; }

void WebGUINode::set_user_map(const cv::Mat& image)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    user_map_ = image.clone();
}

cv::Mat WebGUINode::get_user_map()
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    return user_map_.clone();
}

void WebGUINode::user_map_callback(sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        cv::Mat img(msg->height, msg->width, CV_8UC1, msg->data.data());
        WebGUI::setUserMap(img);
    } catch (...) {}
}

void WebGUINode::odom_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
    last_pose_.x = msg->pose.pose.position.x;
    last_pose_.y = msg->pose.pose.position.y;
    tf2::Quaternion tf_quat;
    tf2::fromMsg(msg->pose.pose.orientation, tf_quat);
    tf2::Matrix3x3 m(tf_quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    last_pose_.yaw = yaw;
}

void WebGUINode::noisy_odom_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
    last_noisy_pose_.x = msg->pose.pose.position.x;
    last_noisy_pose_.y = msg->pose.pose.position.y;
    tf2::Quaternion tf_quat;
    tf2::fromMsg(msg->pose.pose.orientation, tf_quat);
    tf2::Matrix3x3 m(tf_quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    last_noisy_pose_.yaw = yaw;
}

void WebGUI::setUserMap(const cv::Mat& image)
{
    if (image.rows != 970 || image.cols != 1500) {
        throw std::invalid_argument("map passed has the wrong dimensions, it has to be 970 pixels high and 1500 pixels wide");
    }

    cv::Mat processed_image;
    if (image.channels() == 1) {
        cv::cvtColor(image, processed_image, cv::COLOR_GRAY2BGR);
    } else {
        processed_image = image.clone();
    }

    WebGUINode::set_user_map(processed_image);
}

std::vector<double> WebGUI::poseToMap(double x_prime, double y_prime, double yaw_prime)
{
    double y = -23.58 * (-20.36 - x_prime);
    double x = -23.53 * (-31.95 - y_prime);
    double yaw = yaw_prime - M_PI / 2.0;
    return {std::round(x), std::round(y), yaw};
}

std::string WebGUI::base64_encode(unsigned char const* bytes_to_encode, unsigned int in_len)
{
    static const std::string base64_chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";
    std::string ret;
    int i = 0, j = 0;
    unsigned char char_array_3[3], char_array_4[4];
    while (in_len--) {
        char_array_3[i++] = *(bytes_to_encode++);
        if (i == 3) {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;
            for(i = 0; (i < 4); i++) ret += base64_chars[char_array_4[i]];
            i = 0;
        }
    }
    if (i) {
        for(j = i; j < 3; j++) char_array_3[j] = '\0';
        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;
        for (j = 0; (j < i + 1); j++) ret += base64_chars[char_array_4[j]];
        while((i++ < 3)) ret += '=';
    }
    return ret;
}

class session : public std::enable_shared_from_this<session>
{
    tcp::resolver resolver_;
    websocket::stream<beast::tcp_stream> ws_;
    beast::flat_buffer buffer_;
    std::string host_;
    net::steady_timer timer_;
    std::string current_payload_str_;

    bool ack_ = true;
    bool ack_frontend_ = false;
    std::mutex ack_mutex_;

    std::string generate_payload() {
        json payload;
        json payload_img;
        cv::Mat map = WebGUINode::get_user_map();

        if (!map.empty()) {
            std::vector<uchar> buf;
            cv::imencode(".jpg", map, buf);
            std::string b64 = WebGUI::base64_encode(buf.data(), buf.size());
            payload_img["user_map"] = b64;
            payload_img["shape"] = {map.rows, map.cols, map.channels()};
        } else {
            payload_img["user_map"] = nullptr;
            payload_img["shape"] = 0;
        }
        
        payload["user_map"] = payload_img.dump();

        auto rp = WebGUINode::get_pose();
        double rp_x = -23.58 * (-20.36 - rp.x);
        double rp_y = -23.53 * (-31.95 - rp.y);
        std::stringstream rp_ss;
        rp_ss << "(" << rp_x << ", " << rp_y << ", " << rp.yaw << ")";
        payload["real_pose"] = rp_ss.str();

        auto np = WebGUINode::get_noisy_pose();
        double np_x = -23.58 * (-20.36 - np.x);
        double np_y = -23.53 * (-31.95 - np.y);
        std::stringstream np_ss;
        np_ss << "(" << np_x << ", " << np_y << ", " << np.yaw << ")";
        payload["noisy_pose"] = np_ss.str();

        return payload.dump();
    }

public:
    explicit session(net::io_context &ioc) 
        : resolver_(net::make_strand(ioc)), 
          ws_(net::make_strand(ioc)),
          timer_(net::make_strand(ioc)) {}

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
        ws_.async_handshake(host_, "", beast::bind_front_handler(&session::on_handshake, shared_from_this()));
    }

    void on_handshake(beast::error_code ec) {
        if (ec) return;
        do_read();
        
        timer_.expires_after(std::chrono::milliseconds(33));
        timer_.async_wait(beast::bind_front_handler(&session::on_timer, shared_from_this()));
    }

    void do_read() {
        ws_.async_read(buffer_, beast::bind_front_handler(&session::on_read, shared_from_this()));
    }

    void on_read(beast::error_code ec, std::size_t) {
        if (ec) return;
        
        std::string msg = beast::buffers_to_string(buffer_.data());
        buffer_.consume(buffer_.size());
        
        std::lock_guard<std::mutex> lock(ack_mutex_);
        if (msg.find("ack") != std::string::npos) {
            ack_ = true;
        } else if (msg.find("start") != std::string::npos) {
            ack_frontend_ = true;
        }
        
        do_read();
    }

    void on_timer(beast::error_code ec) {
        if (ec) return;
        
        bool should_send = false;
        {
            std::lock_guard<std::mutex> lock(ack_mutex_);
            if (ack_frontend_ && ack_) {
                should_send = true;
                ack_ = false;
            }
        }

        if (should_send) {
            current_payload_str_ = generate_payload();
            ws_.async_write(net::buffer(current_payload_str_), beast::bind_front_handler(&session::on_write, shared_from_this()));
        } else {
            timer_.expires_after(std::chrono::milliseconds(33));
            timer_.async_wait(beast::bind_front_handler(&session::on_timer, shared_from_this()));
        }
    }

    void on_write(beast::error_code ec, std::size_t) {
        if (ec) return;
        timer_.expires_after(std::chrono::milliseconds(33));
        timer_.async_wait(beast::bind_front_handler(&session::on_timer, shared_from_this()));
    }
};

WebGUI::WebGUI()
{
    net::io_context ioc;
    std::make_shared<session>(ioc)->run("127.0.0.1", "2303");
    ioc.run();
}