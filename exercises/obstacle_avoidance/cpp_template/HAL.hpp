#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include <thread>
#include <vector>

using namespace std::chrono_literals; // NOLINT
using namespace std;                  // NOLINT
using std::placeholders::_1;

// ---------------------------------------------------------------------------
// Data structures exposed to students
// ---------------------------------------------------------------------------

struct Pose3d
{
  double x   = 0.0;
  double y   = 0.0;
  double yaw = 0.0;
};

struct LaserData
{
  std::vector<float> values;
  float min_angle       = 0.0f;
  float max_angle       = 0.0f;
  float angle_increment = 0.0f;
  float max_range       = 0.0f;
};

// ---------------------------------------------------------------------------
// Public HAL API — called from student code (academy.cpp)
// ---------------------------------------------------------------------------

namespace HAL
{
  void set_v(float speed);
  void set_w(float speed);
  Pose3d    get_pose3d();
  LaserData get_laser_data();
}  // namespace HAL

// ---------------------------------------------------------------------------
// Internal implementation
// ---------------------------------------------------------------------------

namespace
{

// File-scope state
geometry_msgs::msg::Twist g_last_twist;
Pose3d    g_last_pose3d;
LaserData g_last_laser;

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr g_vel_pub;

class HALNode : public rclcpp::Node
{
public:
  HALNode() : Node("hal_node")
  {
    g_vel_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&HALNode::odom_callback, this, _1));

    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/f1/laser/scan", 10,
        std::bind(&HALNode::laser_callback, this, _1));
  }

private:
  void odom_callback(nav_msgs::msg::Odometry::UniquePtr msg)
  {
    double roll, pitch, yaw;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    g_last_pose3d.x   = msg->pose.pose.position.x;
    g_last_pose3d.y   = msg->pose.pose.position.y;
    g_last_pose3d.yaw = yaw;
  }

  void laser_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
  {
    g_last_laser.values.assign(msg->ranges.begin(), msg->ranges.end());
    g_last_laser.min_angle       = msg->angle_min;
    g_last_laser.max_angle       = msg->angle_max;
    g_last_laser.angle_increment = msg->angle_increment;
    g_last_laser.max_range       = msg->range_max;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
};

std::shared_ptr<HALNode> g_hal_node;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> g_executor;
std::thread g_spin_thread;

}  // anonymous namespace

// ---------------------------------------------------------------------------
// hal_init() — called once from main() before spawning student thread
// ---------------------------------------------------------------------------

inline void hal_init()
{
  g_hal_node = std::make_shared<HALNode>();
  g_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 2);
  g_executor->add_node(g_hal_node);
  g_spin_thread = std::thread([] { g_executor->spin(); });
  g_spin_thread.detach();
}

// ---------------------------------------------------------------------------
// HAL namespace — public API implementations
// ---------------------------------------------------------------------------

namespace HAL
{

inline void set_v(float speed)
{
  g_last_twist.linear.x = speed;
  if (g_vel_pub) g_vel_pub->publish(g_last_twist);
}

inline void set_w(float speed)
{
  g_last_twist.angular.z = speed;
  if (g_vel_pub) g_vel_pub->publish(g_last_twist);
}

inline Pose3d get_pose3d()
{
  return g_last_pose3d;
}

inline LaserData get_laser_data()
{
  return g_last_laser;
}

}  // namespace HAL

#endif  // INCLUDE_HAL_HPP_
