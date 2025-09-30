#ifndef INCLUDE_WEBGUI_HPP_
#define INCLUDE_WEBGUI_HPP_

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <iostream>
#include <thread>
#include <mutex>

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;
using namespace std;

// int main()
// {
//     try
//     {
//         net::io_context ioc;
//         tcp::resolver resolver{ioc};
//         websocket::stream<tcp::socket> ws{ioc};

//         auto const results = resolver.resolve("localhost", "2303");
//         auto ep = net::connect(ws.next_layer(), results);
//         ws.handshake(ep.address().to_string() + ":2303", "/");

//         thread on_message();

//         string message = "Hello, WebSocket server!";
//         ws.write(net::buffer(message));

//         beast::flat_buffer buffer;
//         ws.read(buffer);
//         cout << "Received: " << beast::make_printable(buffer.data()) << endl;

//         ws.close(websocket::close_code::normal);
//     }
//     catch (std::exception const &e)
//     {
//         std::cerr << "Exception: " << e.what() << std::endl;
//     }
//     return 0;
// }

class WebGUI
{
private:
    static websocket::stream<tcp::socket> ws;

    static void on_message();

public:
    WebGUI(string host);
    ~WebGUI();

    static void show_image(string image);
};

WebGUI::WebGUI(string host = "2303")
{
    net::io_context ioc;
    tcp::resolver resolver{ioc};
    websocket::stream<tcp::socket> websocket{ioc};

    auto const results = resolver.resolve("ws://127.0.0.1", host);
    auto ep = net::connect(websocket.next_layer(), results);
    websocket.handshake(ep.address().to_string() + ":" + host, "/");
    WebGUI::ws = websocket;
    
    thread t1(WebGUI::on_message);
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
