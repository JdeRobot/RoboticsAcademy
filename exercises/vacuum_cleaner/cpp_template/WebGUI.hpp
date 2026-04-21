#ifndef INCLUDE_WEBGUI_HPP_
#define INCLUDE_WEBGUI_HPP_

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <boost/asio/strand.hpp>
#include "json.hpp"
#include "Frequency.hpp"
#include <utility>
#include <iomanip>
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "gazebo_msgs/msg/performance_metrics.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;
using namespace std;
using json = nlohmann::json;

class WebGUI
{
private:
public:
    WebGUI();

    static string img_payload;
};

using namespace std::chrono_literals; // NOLINT
using namespace std;                  // NOLINT
using std::placeholders::_1;

class WebGUINode : public rclcpp::Node
{
public:
    WebGUINode() : Node("webgui_node")
    {
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&WebGUINode::pose_callback, this, _1));
        perf_sub_ = create_subscription<gazebo_msgs::msg::PerformanceMetrics>(
            "/performance_metrics", 10,
            std::bind(&WebGUINode::performance_callback, this, _1));
    };

    static vector<double> get_pose()
    {
        tf2::Quaternion tf_quat;
        tf2::fromMsg(last_odom.pose.pose.orientation, tf_quat); //Assume quat_msg is a quaternion ros msg

        tf2::Matrix3x3 m(tf_quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        const int scale_x = -30;
        const int offset_x = 171;
        const double x = scale_x * last_odom.pose.pose.position.x + offset_x;

        const int scale_y = 15;
        const int offset_y = 63;
        const double y = scale_y * last_odom.pose.pose.position.y + offset_y;

        vector<double> v = {x, y, yaw};
        return v;
    };

    static double get_performance()
    {
        return last_perf.real_time_factor;
    };

private:
    void pose_callback(nav_msgs::msg::Odometry::UniquePtr msg)
    {
        last_odom = *msg;
    };

    void performance_callback(gazebo_msgs::msg::PerformanceMetrics::UniquePtr msg)
    {
        last_perf = *msg;
    };

    // Publisher
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<gazebo_msgs::msg::PerformanceMetrics>::SharedPtr perf_sub_;

    // Message
    static nav_msgs::msg::Odometry last_odom;
    static gazebo_msgs::msg::PerformanceMetrics last_perf;
};

nav_msgs::msg::Odometry WebGUINode::last_odom = nav_msgs::msg::Odometry();
gazebo_msgs::msg::PerformanceMetrics WebGUINode::last_perf = gazebo_msgs::msg::PerformanceMetrics();

// Report a failure
void fail(beast::error_code ec, char const *what)
{
    std::cerr << what << ": " << ec.message() << "\n";
}

// Sends a WebSocket message and prints the response
class session : public std::enable_shared_from_this<session>
{
    tcp::resolver resolver_;
    websocket::stream<beast::tcp_stream> ws_;
    beast::flat_buffer buffer_;
    std::string host_;
    std::string text_;

public:
    // Resolver and socket require an io_context
    explicit session(net::io_context &ioc)
        : resolver_(net::make_strand(ioc)), ws_(net::make_strand(ioc))
    {
    }

    // Start the asynchronous operation
    void
    run(
        char const *host,
        char const *port,
        char const *text)
    {
        // Save these for later
        host_ = host;
        text_ = text;
        buffer_.max_size(1024 * 1024);

        // Look up the domain name
        resolver_.async_resolve(
            host,
            port,
            beast::bind_front_handler(
                &session::on_resolve,
                shared_from_this()));
    }

    void
    on_resolve(
        beast::error_code ec,
        tcp::resolver::results_type results)
    {
        if (ec)
            return fail(ec, "resolve");

        // Set the timeout for the operation
        beast::get_lowest_layer(ws_).expires_after(std::chrono::seconds(30));

        // Make the connection on the IP address we get from a lookup
        beast::get_lowest_layer(ws_).async_connect(
            results,
            beast::bind_front_handler(
                &session::on_connect,
                shared_from_this()));
    }

    void
    on_connect(beast::error_code ec, tcp::resolver::results_type::endpoint_type ep)
    {
        if (ec)
            return fail(ec, "connect");

        // Turn off the timeout on the tcp_stream, because
        // the websocket stream has its own timeout system.
        beast::get_lowest_layer(ws_).expires_never();

        // Set suggested timeout settings for the websocket
        ws_.set_option(
            websocket::stream_base::timeout::suggested(
                beast::role_type::client));

        // Set a decorator to change the User-Agent of the handshake
        ws_.read_message_max(1024 * 1024);
        ws_.auto_fragment(false);
        ws_.set_option(websocket::stream_base::decorator(
            [](websocket::request_type &req)
            {
                req.set(http::field::user_agent,
                        std::string(BOOST_BEAST_VERSION_STRING) +
                            " websocket-client-async");
            }));

        // Update the host_ string. This will provide the value of the
        // Host HTTP header during the WebSocket handshake.
        // See https://tools.ietf.org/html/rfc7230#section-5.4
        host_ += ':' + std::to_string(ep.port());

        // Perform the websocket handshake
        ws_.async_handshake(host_, "",
                            beast::bind_front_handler(
                                &session::on_handshake,
                                shared_from_this()));
    }

    void
    on_handshake(beast::error_code ec)
    {
        if (ec)
            return fail(ec, "handshake");

        // Send the message
        ws_.async_write(
            net::buffer(text_),
            beast::bind_front_handler(
                &session::on_write,
                shared_from_this()));
    }

    void
    on_write(
        beast::error_code ec,
        std::size_t bytes_transferred)
    {
        boost::ignore_unused(bytes_transferred);

        if (ec)
            return fail(ec, "write");

        // Read a message into our buffer
        ws_.async_read(
            buffer_,
            beast::bind_front_handler(
                &session::on_read,
                shared_from_this()));
    }

    void
    on_read(
        beast::error_code ec,
        std::size_t bytes_transferred)
    {
        boost::ignore_unused(bytes_transferred);

        if (ec)
            return fail(ec, "read");

        ws_.text(ws_.got_text());
        unsigned char *cp = (unsigned char *)buffer_.data().data();
        string msg(reinterpret_cast<char const *>(cp));

        buffer_.consume(buffer_.size());

        auto pose = WebGUINode::get_pose();
        const json map = json{pose.at(0), pose.at(1), pose.at(2)};
        double rtf = WebGUINode::get_performance();
        std::stringstream stream;
        stream << std::fixed << std::setprecision(2) << rtf;
        std::string rtf_str = stream.str();
#ifdef USER_NODE
        const json j = json{{"map", map.dump()}, {"brain", Frequency::rate}, {"gui", 20}, {"rtf", rtf_str}, {"fps", -1}, {"lat", -1}};
#else
        const json j = json{{"map", map.dump()}, {"brain", Frequency::rate}, {"gui", 20}, {"rtf", rtf_str}, {"fps", -1}, {"lat", -1}};
#endif
        auto const text = j.dump();

        // Close the WebSocket connection
        ws_.async_write(
            net::buffer(text.c_str(), strlen(text.c_str())),
            beast::bind_front_handler(
                &session::on_write,
                shared_from_this()));
    }

    void
    on_close(beast::error_code ec)
    {
        if (ec)
            return fail(ec, "close");

        // If we get here then the connection is closed gracefully

        // The make_printable() function helps print a ConstBufferSequence
        std::cout << beast::make_printable(buffer_.data()) << std::endl;
    }
};

WebGUI::WebGUI()
{
    auto const host = "127.0.0.1";
    auto const port = "2303";
    const json j = json{{"map", "(201,85.5,0)"}};
    auto const text = j.dump();

    net::io_context ioc;

    // Launch the asynchronous operation
    std::make_shared<session>(ioc)->run(host, port, text.c_str());

    ioc.run();
}

#endif
