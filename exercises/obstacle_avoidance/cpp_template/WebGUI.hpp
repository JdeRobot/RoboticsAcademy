#ifndef INCLUDE_WEBGUI_HPP_
#define INCLUDE_WEBGUI_HPP_

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>
#include <array>
#include <cstdlib>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "Frequency.hpp"
#include "HAL.hpp"
#include "Lap.hpp"
#include "json.hpp"

namespace beast     = boost::beast;
namespace http      = beast::http;
namespace websocket = beast::websocket;
namespace net       = boost::asio;
using tcp           = net::ip::tcp;
using namespace std; // NOLINT
using json = nlohmann::json;

// ---------------------------------------------------------------------------
// Target — waypoint that the student navigates to
// ---------------------------------------------------------------------------

struct Target
{
  std::string id;
  double x;
  double y;
  bool reached = false;

  const std::string & getId()   const { return id; }
  double              getX()    const { return x; }
  double              getY()    const { return y; }
  bool                isReached() const { return reached; }
  void                setReached(bool v) { reached = v; }
};

// ---------------------------------------------------------------------------
// WebGUI class — public interface used from academy.cpp
// ---------------------------------------------------------------------------

class WebGUI
{
public:
  WebGUI();

  // Display the three force vectors on the map panel.
  // vec1 = car/attractive force (x, y)
  // vec2 = obstacle/repulsive force (x, y)
  // vec3 = average/result force (x, y)
  static void show_forces(std::array<double, 2> car,
                          std::array<double, 2> obs,
                          std::array<double, 2> avg);

  // Set the local sub-target that is currently being pursued.
  static void show_local_target(double x, double y);

  // Returns a reference to the current (first unreached) target.
  // When all targets have been reached they are reset and the first
  // target is returned again.
  static Target & get_next_target();

  // Force-vector components (read by the WebSocket session)
  static double car_x, car_y;
  static double obs_x, obs_y;
  static double avg_x, avg_y;

  // Local sub-target position
  static double target_x, target_y;

  // Waypoint list loaded at startup
  static std::vector<Target> targets;
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static void fail_oa(beast::error_code ec, char const * what)
{
  std::cerr << what << ": " << ec.message() << "\n";
}

// Build the "laser" array and return max_range expected by the frontend.
// Each element: [scaled_dist, angle_rad]
// dist is multiplied by 20 to match the Python template scale factor.
// Infinite readings are capped at max_range.
static json build_laser_json(float & out_max_range)
{
  LaserData ld = HAL::get_laser_data();
  json arr     = json::array();

  int n = static_cast<int>(ld.values.size());
  if (n == 0)
  {
    out_max_range = 1000.0f;
    return arr;
  }

  // Use the sensor-reported increment directly so beam angles match
  // Python's math.radians(i) convention (1 deg/beam when increment == pi/180).
  float inc = ld.angle_increment;

  for (int i = 0; i < n; i++)
  {
    float dist = 20.0f * ld.values[i];
    float ang  = ld.min_angle + static_cast<float>(i) * inc;

    if (!std::isfinite(dist) || dist > 20.0f * ld.max_range)
    {
      dist = 20.0f * ld.max_range;
    }

    arr.push_back(json::array({dist, ang}));
  }

  out_max_range = 20.0f * ld.max_range;
  return arr;
}

// ---------------------------------------------------------------------------
// WebSocket session
// ---------------------------------------------------------------------------

class session_oa : public std::enable_shared_from_this<session_oa>
{
  tcp::resolver resolver_;
  websocket::stream<beast::tcp_stream> ws_;
  beast::flat_buffer buffer_;
  std::string host_;
  std::string text_;
  Lap * lap_;

public:
  explicit session_oa(net::io_context & ioc)
      : resolver_(net::make_strand(ioc)), ws_(net::make_strand(ioc))
  {
  }

  void run(char const * host, char const * port, char const * text, Lap * lap)
  {
    host_ = host;
    text_ = text;
    lap_  = lap;
    buffer_.max_size(1024 * 1024);

    resolver_.async_resolve(
        host, port,
        beast::bind_front_handler(&session_oa::on_resolve, shared_from_this()));
  }

  void on_resolve(beast::error_code ec, tcp::resolver::results_type results)
  {
    if (ec) return fail_oa(ec, "resolve");

    beast::get_lowest_layer(ws_).expires_after(std::chrono::seconds(30));
    beast::get_lowest_layer(ws_).async_connect(
        results,
        beast::bind_front_handler(&session_oa::on_connect, shared_from_this()));
  }

  void on_connect(beast::error_code ec, tcp::resolver::results_type::endpoint_type ep)
  {
    if (ec) return fail_oa(ec, "connect");

    beast::get_lowest_layer(ws_).expires_never();
    ws_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::client));
    ws_.read_message_max(1024 * 1024);
    ws_.auto_fragment(false);
    ws_.set_option(websocket::stream_base::decorator(
        [](websocket::request_type & req)
        {
          req.set(http::field::user_agent,
                  std::string(BOOST_BEAST_VERSION_STRING) + " websocket-client-async");
        }));

    host_ += ':' + std::to_string(ep.port());
    ws_.async_handshake(
        host_, "",
        beast::bind_front_handler(&session_oa::on_handshake, shared_from_this()));
  }

  void on_handshake(beast::error_code ec)
  {
    if (ec) return fail_oa(ec, "handshake");

    ws_.async_write(
        net::buffer(text_),
        beast::bind_front_handler(&session_oa::on_write, shared_from_this()));
  }

  void on_write(beast::error_code ec, std::size_t bytes_transferred)
  {
    boost::ignore_unused(bytes_transferred);
    if (ec) return fail_oa(ec, "write");

    ws_.async_read(
        buffer_,
        beast::bind_front_handler(&session_oa::on_read, shared_from_this()));
  }

  void on_read(beast::error_code ec, std::size_t bytes_transferred)
  {
    boost::ignore_unused(bytes_transferred);
    if (ec) return fail_oa(ec, "read");

    ws_.text(ws_.got_text());

    // Handle control messages from the server (pause / start)
    unsigned char * cp  = static_cast<unsigned char *>(buffer_.data().data());
    string          msg(reinterpret_cast<char const *>(cp));

    if (msg == "pause")       lap_->pause();
    else if (msg == "start")  lap_->start();

    buffer_.consume(buffer_.size());

    // -----------------------------------------------------------------------
    // Build the map JSON (inner payload, JSON-stringified for the frontend)
    // -----------------------------------------------------------------------
    Pose3d pose = HAL::get_pose3d();

    float out_max_range = 1000.0f;
    json  laser_arr     = build_laser_json(out_max_range);

    const json map_data = {
        {"pose",     json::array({pose.x, pose.y, pose.yaw})},
        {"target",   json::array({WebGUI::target_x, WebGUI::target_y})},
        {"car",      json::array({WebGUI::car_x,    WebGUI::car_y})},
        {"obstacle", json::array({WebGUI::obs_x,    WebGUI::obs_y})},
        {"average",  json::array({WebGUI::avg_x,    WebGUI::avg_y})},
        {"laser",    laser_arr},
        {"max_range", out_max_range}};

    // -----------------------------------------------------------------------
    // Outer message sent to the frontend
    // -----------------------------------------------------------------------
    const json j = {
        {"map",   map_data.dump()},
        {"lap",   lap_->getLapTime()},
        {"brain", Frequency::rate},
        {"gui",   20},
        {"fps",   -1},
        {"lat",   -1}};

    const string text = j.dump();

    ws_.async_write(
        net::buffer(text.c_str(), strlen(text.c_str())),
        beast::bind_front_handler(&session_oa::on_write, shared_from_this()));
  }

  void on_close(beast::error_code ec)
  {
    if (ec) return fail_oa(ec, "close");
    std::cout << beast::make_printable(buffer_.data()) << std::endl;
  }
};

// ---------------------------------------------------------------------------
// WebGUI constructor — starts the WebSocket event loop (blocks main thread)
// ---------------------------------------------------------------------------

WebGUI::WebGUI()
{
  auto const host = "127.0.0.1";
  auto const port = "2303";

  // Initial handshake payload: empty map
  const json init_map = {
      {"pose",      json::array({0.0, 0.0, 0.0})},
      {"target",    json::array({0.0, 0.0})},
      {"car",       json::array({2.0, 0.0})},
      {"obstacle",  json::array({0.0, 2.0})},
      {"average",   json::array({-2.0, 0.0})},
      {"laser",     json::array()},
      {"max_range", 1000.0}};
  const json j    = {{"map", init_map.dump()}, {"lap", "0:0:0.0"}};
  const string text = j.dump();

  Lap * lap = new Lap();

  net::io_context ioc;
  std::make_shared<session_oa>(ioc)->run(host, port, text.c_str(), lap);
  ioc.run();
}

// ---------------------------------------------------------------------------
// WebGUI static method implementations
// ---------------------------------------------------------------------------

void WebGUI::show_forces(std::array<double, 2> car,
                         std::array<double, 2> obs,
                         std::array<double, 2> avg)
{
  car_x = car[0]; car_y = car[1];
  obs_x = obs[0]; obs_y = obs[1];
  avg_x = avg[0]; avg_y = avg[1];
}

void WebGUI::show_local_target(double x, double y)
{
  target_x = x;
  target_y = y;
}

Target & WebGUI::get_next_target()
{
  for (auto & t : targets)
  {
    if (!t.isReached())
    {
      // Mirror Python map.getNextTarget(): auto-update the displayed target
      target_x = t.x;
      target_y = t.y;
      return t;
    }
  }
  // All targets reached — reset and return first (mirrors Python resetTargets())
  for (auto & t : targets) t.setReached(false);
  target_x = targets[0].x;
  target_y = targets[0].y;
  return targets[0];
}

// ---------------------------------------------------------------------------
// Static member definitions
// ---------------------------------------------------------------------------

double WebGUI::car_x = 2.0;
double WebGUI::car_y = 0.0;
double WebGUI::obs_x = 0.0;
double WebGUI::obs_y = 2.0;
double WebGUI::avg_x = -2.0;
double WebGUI::avg_y = 0.0;
double WebGUI::target_x = 0.0;
double WebGUI::target_y = 0.0;

// Waypoints matching targets_f1.json
std::vector<Target> WebGUI::targets = {
    {"target01",    0.0,  -30.0},
    {"target02",   -6.0,  -41.0},
    {"target03",  -14.0,  -31.0},
    {"target04",  -16.0,  -13.0},
    {"target05",  -55.0,  -12.0},
    {"target06",  -63.0,  -30.0},
    {"target07",  -72.0,  -35.0},
    {"target08", -106.0,   -8.0},
    {"target09",  -68.0,   38.0},
    {"target10",  -47.0,   47.0},
    {"target11",  -38.0,   64.0},
    {"target12",  -26.0,   43.0},
    {"target13",  -17.0,   58.0},
    {"target14",   -3.0,   58.0},
    {"target15",    0.0,   13.0},
};

#endif  // INCLUDE_WEBGUI_HPP_
