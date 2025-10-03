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

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;
using namespace std;

class WebGUI
{
private:
    static websocket::stream<tcp::socket> ws;

    static void on_message();

public:
    WebGUI(string port);
    ~WebGUI();

    static void show_image(string image);
};

WebGUI::WebGUI(string port = "2303")
{
    std::string host = "127.0.0.1";
    auto const port2 = port;
    auto const text = "{\"gui\":\"3\"}";

    // The io_context is required for all I/O
    net::io_context ioc;

    // These objects perform our I/O
    tcp::resolver resolver{ioc};
    websocket::stream<tcp::socket> ws{ioc};

    // Look up the domain name
    auto const results = resolver.resolve(host, port2);

    // Make the connection on the IP address we get from a lookup
    auto ep = net::connect(ws.next_layer(), results);

    // Update the host_ string. This will provide the value of the
    // Host HTTP header during the WebSocket handshake.
    // See https://tools.ietf.org/html/rfc7230#section-5.4
    host += ':' + std::to_string(ep.port());

    // Set a decorator to change the User-Agent of the handshake
    ws.set_option(websocket::stream_base::decorator(
        [](websocket::request_type &req)
        {
            req.set(http::field::user_agent,
                    std::string(BOOST_BEAST_VERSION_STRING) +
                        " websocket-client-coro");
        }));

    // Perform the websocket handshake
    ws.handshake(host, "/");

    // Send the message
    ws.write(net::buffer(std::string(text)));
    // net::io_context ioc;
    // tcp::resolver resolver{ioc};
    // websocket::stream<tcp::socket> websocket{ioc};

    // auto const results = resolver.resolve("ws://127.0.0.1", host);
    // auto ep = net::connect(websocket.next_layer(), results);
    // websocket.handshake(ep.address().to_string() + ":" + host, "/");
    // WebGUI::ws = websocket;

    // thread t1(WebGUI::on_message);
}

WebGUI::~WebGUI()
{
    ws.close(websocket::close_code::normal);
}

void WebGUI::on_message()
{
    for (;;)
    {
        beast::flat_buffer buffer;
        ws.read(buffer);
        std::cout << "Received: " << beast::make_printable(buffer.data()) << std::endl;
    }
}

void WebGUI::show_image(string image)
{
    ws.write(net::buffer(image));
}

net::io_context ioc;
websocket::stream<tcp::socket> WebGUI::ws{ioc};

#endif
