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

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;
using namespace std;

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

        // Close the WebSocket connection
        ws_.async_write(
            net::buffer(text_),
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


const char kBase64Alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
"abcdefghijklmnopqrstuvwxyz"
"0123456789+/";

class Base64 {
public:
    static bool Encode(const std::string &in, std::string *out) {
        int i = 0, j = 0;
        size_t enc_len = 0;
        unsigned char a3[3];
        unsigned char a4[4];

        out->resize(EncodedLength(in));

        int input_len = in.size();
        std::string::const_iterator input = in.begin();

        while (input_len--) {
            a3[i++] = *(input++);
            if (i == 3) {
                a3_to_a4(a4, a3);

                for (i = 0; i < 4; i++) {
                    (*out)[enc_len++] = kBase64Alphabet[a4[i]];
                }

                i = 0;
            }
        }

        if (i) {
            for (j = i; j < 3; j++) {
                a3[j] = '\0';
            }

            a3_to_a4(a4, a3);

            for (j = 0; j < i + 1; j++) {
                (*out)[enc_len++] = kBase64Alphabet[a4[j]];
            }

            while ((i++ < 3)) {
                (*out)[enc_len++] = '=';
            }
        }

        return (enc_len == out->size());
    }

    static bool Encode(const char *input, size_t input_length, char *out, size_t out_length) {
        int i = 0, j = 0;
        char *out_begin = out;
        unsigned char a3[3];
        unsigned char a4[4];

        size_t encoded_length = EncodedLength(input_length);

        if (out_length < encoded_length) return false;

        while (input_length--) {
            a3[i++] = *input++;
            if (i == 3) {
                a3_to_a4(a4, a3);

                for (i = 0; i < 4; i++) {
                    *out++ = kBase64Alphabet[a4[i]];
                }

                i = 0;
            }
        }

        if (i) {
            for (j = i; j < 3; j++) {
                a3[j] = '\0';
            }

            a3_to_a4(a4, a3);

            for (j = 0; j < i + 1; j++) {
                *out++ = kBase64Alphabet[a4[j]];
            }

            while ((i++ < 3)) {
                *out++ = '=';
            }
        }

        return (out == (out_begin + encoded_length));
    }

    static bool Decode(const std::string &in, std::string *out) {
        int i = 0, j = 0;
        size_t dec_len = 0;
        unsigned char a3[3];
        unsigned char a4[4];

        int input_len = in.size();
        std::string::const_iterator input = in.begin();

        out->resize(DecodedLength(in));

        while (input_len--) {
            if (*input == '=') {
                break;
            }

            a4[i++] = *(input++);
            if (i == 4) {
                for (i = 0; i < 4; i++) {
                    a4[i] = b64_lookup(a4[i]);
                }

                a4_to_a3(a3, a4);

                for (i = 0; i < 3; i++) {
                    (*out)[dec_len++] = a3[i];
                }

                i = 0;
            }
        }

        if (i) {
            for (j = i; j < 4; j++) {
                a4[j] = '\0';
            }

            for (j = 0; j < 4; j++) {
                a4[j] = b64_lookup(a4[j]);
            }

            a4_to_a3(a3, a4);

            for (j = 0; j < i - 1; j++) {
                (*out)[dec_len++] = a3[j];
            }
        }

        return (dec_len == out->size());
    }

    static bool Decode(const char *input, size_t input_length, char *out, size_t out_length) {
        int i = 0, j = 0;
        char *out_begin = out;
        unsigned char a3[3];
        unsigned char a4[4];

        size_t decoded_length = DecodedLength(input, input_length);

        if (out_length < decoded_length) return false;

        while (input_length--) {
            if (*input == '=') {
                break;
            }

            a4[i++] = *(input++);
            if (i == 4) {
                for (i = 0; i < 4; i++) {
                    a4[i] = b64_lookup(a4[i]);
                }

                a4_to_a3(a3, a4);

                for (i = 0; i < 3; i++) {
                    *out++ = a3[i];
                }

                i = 0;
            }
        }

        if (i) {
            for (j = i; j < 4; j++) {
                a4[j] = '\0';
            }

            for (j = 0; j < 4; j++) {
                a4[j] = b64_lookup(a4[j]);
            }

            a4_to_a3(a3, a4);

            for (j = 0; j < i - 1; j++) {
                *out++ = a3[j];
            }
        }

        return (out == (out_begin + decoded_length));
    }

    static int DecodedLength(const char *in, size_t in_length) {
        int numEq = 0;

        const char *in_end = in + in_length;
        while (*--in_end == '=') ++numEq;

        return ((6 * in_length) / 8) - numEq;
    }

    static int DecodedLength(const std::string &in) {
        int numEq = 0;
        int n = in.size();

        for (std::string::const_reverse_iterator it = in.rbegin(); *it == '='; ++it) {
            ++numEq;
        }

        return ((6 * n) / 8) - numEq;
    }

    inline static int EncodedLength(size_t length) {
        return (length + 2 - ((length + 2) % 3)) / 3 * 4;
    }

    inline static int EncodedLength(const std::string &in) {
        return EncodedLength(in.length());
    }

    inline static void StripPadding(std::string *in) {
        while (!in->empty() && *(in->rbegin()) == '=') in->resize(in->size() - 1);
    }

private:
    static inline void a3_to_a4(unsigned char * a4, unsigned char * a3) {
        a4[0] = (a3[0] & 0xfc) >> 2;
        a4[1] = ((a3[0] & 0x03) << 4) + ((a3[1] & 0xf0) >> 4);
        a4[2] = ((a3[1] & 0x0f) << 2) + ((a3[2] & 0xc0) >> 6);
        a4[3] = (a3[2] & 0x3f);
    }

    static inline void a4_to_a3(unsigned char * a3, unsigned char * a4) {
        a3[0] = (a4[0] << 2) + ((a4[1] & 0x30) >> 4);
        a3[1] = ((a4[1] & 0xf) << 4) + ((a4[2] & 0x3c) >> 2);
        a3[2] = ((a4[2] & 0x3) << 6) + a4[3];
    }

    static inline unsigned char b64_lookup(unsigned char c) {
        if (c >= 'A' && c <= 'Z') return c - 'A';
        if (c >= 'a' && c <= 'z') return c - 71;
        if (c >= '0' && c <= '9') return c + 4;
        if (c == '+') return 62;
        if (c == '/') return 63;
        return 255;
    }
};

class WebGUI
{
private:

public:
    WebGUI();

    static char * img_payload;
    static void show_image(cv::Mat image);
};

WebGUI::WebGUI()
{
    auto const host = "127.0.0.1";
    auto const port = "2303";
    auto const text = "{\"gui\":{\"map\":\"(3,3)\", \"img\":}}";

    net::io_context ioc;

    // Launch the asynchronous operation
    std::make_shared<session>(ioc)->run(host, port, text);

    // Run the I/O service. The call will return when
    // the get operation is complete.
    ioc.run();
}

void WebGUI::show_image(cv::Mat image)
{
    if (image.empty()) {
        return;
    }

    vector<uchar> buf;
    cv::imencode(".png", image, buf);
    int width = image.cols;
    int height = image.rows;
    auto base64_png = reinterpret_cast<const char *>(buf.data());
    string raw_img = base64_png;
    string encoded_png;
    Base64::Encode(raw_img, &encoded_png);
    string payload = "\"img\":{\"image\": \"" + encoded_png + "\", \"shape\": [" + to_string(width) + ", " + to_string(height) + "]}";
    img_payload = const_cast<char*>(payload.c_str());
    cout << img_payload << endl;
}

char * WebGUI::img_payload = "\"img\":{\"image\": \"\", \"shape\": \"\"}";

#endif
