#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <iostream>

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

int main() {
    try {
        net::io_context ioc;
        tcp::resolver resolver{ioc};
        websocket::stream<tcp::socket> ws{ioc};

        auto const results = resolver.resolve("localhost", "2303");
        auto ep = net::connect(ws.next_layer(), results);
        ws.handshake(ep.address().to_string() + ":2303", "/");

        std::string message = "Hello, WebSocket server!";
        ws.write(net::buffer(message));

        beast::flat_buffer buffer;
        ws.read(buffer);
        std::cout << "Received: " << beast::make_printable(buffer.data()) << std::endl;

        ws.close(websocket::close_code::normal);
    } catch(std::exception const& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
    return 0;
}