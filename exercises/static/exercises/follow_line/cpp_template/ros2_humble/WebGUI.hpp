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
#include "opencv2/opencv.hpp"
#include "json.hpp"
#include "HAL.hpp"
#include "Frequency.hpp"
#include "Lap.hpp"
#include <utility>
#include <iomanip>
#include <sstream>

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
    static void show_image(cv::Mat image);
};

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
    Lap *lap_;

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
        char const *text,
        Lap *lap)
    {
        // Save these for later
        host_ = host;
        text_ = text;
        lap_ = lap;
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

        if (msg == "pause")
        {
            lap_->pause();
        }
        else if (msg == "start")
        {
            lap_->start();
        }

        buffer_.consume(buffer_.size());

        auto pose = HAL::get_pose();
        const json map = json{pose.at(0), pose.at(1)};
        double rtf = HAL::get_performance();
        std::stringstream stream;
        stream << std::fixed << std::setprecision(2) << rtf;
        std::string rtf_str = stream.str();
        const json j = json{{"map", map.dump()}, {"image", WebGUI::img_payload}, {"lap", lap_->getLapTime()}, {"brain", Frequency::rate}, {"gui", 20}, {"rtf", rtf_str}, {"fps", -1}, {"lat", -1}};
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

static const char *base64_chars[2] = {
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789"
    "+/",

    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789"
    "-_"};

string base64_encode(unsigned char const *bytes_to_encode, size_t in_len, bool url)
{

    size_t len_encoded = (in_len + 2) / 3 * 4;

    unsigned char trailing_char = url ? '.' : '=';

    //
    // Choose set of base64 characters. They differ
    // for the last two positions, depending on the url
    // parameter.
    // A bool (as is the parameter url) is guaranteed
    // to evaluate to either 0 or 1 in C++ therefore,
    // the correct character set is chosen by subscripting
    // base64_chars with url.
    //
    const char *base64_chars_ = base64_chars[url];

    std::string ret;
    ret.reserve(len_encoded);

    unsigned int pos = 0;

    while (pos < in_len)
    {
        ret.push_back(base64_chars_[(bytes_to_encode[pos + 0] & 0xfc) >> 2]);

        if (pos + 1 < in_len)
        {
            ret.push_back(base64_chars_[((bytes_to_encode[pos + 0] & 0x03) << 4) + ((bytes_to_encode[pos + 1] & 0xf0) >> 4)]);

            if (pos + 2 < in_len)
            {
                ret.push_back(base64_chars_[((bytes_to_encode[pos + 1] & 0x0f) << 2) + ((bytes_to_encode[pos + 2] & 0xc0) >> 6)]);
                ret.push_back(base64_chars_[bytes_to_encode[pos + 2] & 0x3f]);
            }
            else
            {
                ret.push_back(base64_chars_[(bytes_to_encode[pos + 1] & 0x0f) << 2]);
                ret.push_back(trailing_char);
            }
        }
        else
        {

            ret.push_back(base64_chars_[(bytes_to_encode[pos + 0] & 0x03) << 4]);
            ret.push_back(trailing_char);
            ret.push_back(trailing_char);
        }

        pos += 3;
    }

    return ret;
}

WebGUI::WebGUI()
{
    auto const host = "127.0.0.1";
    auto const port = "2303";
    const json image = json{{"image", ""}, {"shape", {100, 100}}};
    const json j = json{{"map", "(54,-12)"}, {"image", image.dump()}, {"lap", "0:0:0.0"}};
    auto const text = j.dump();
    Lap *lap = new Lap();

    net::io_context ioc;

    // Launch the asynchronous operation
    std::make_shared<session>(ioc)->run(host, port, text.c_str(), lap);

    ioc.run();
}

void WebGUI::show_image(cv::Mat image)
{
    if (image.empty())
    {
        return;
    }

    vector<uchar> buf;
    cv::imencode(".png", image, buf);
    int width = image.cols;
    int height = image.rows;
    unsigned char const *enc_msg = reinterpret_cast<unsigned char *>(buf.data());
    string encoded = base64_encode(enc_msg, buf.size(), false);
    img_payload = json{{"image", encoded}, {"shape", {width, height}}}.dump();
}

string WebGUI::img_payload = json{{"image", ""}, {"shape", {0, 0}}}.dump();

#endif
