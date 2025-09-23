#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals; // NOLINT
using std::placeholders::_1;

class FollowLineNode : public rclcpp::Node
{
public:
  FollowLineNode();

private:
  // Control cycle
  void control_cycle();

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  // Message
  geometry_msgs::msg::Twist out_vel_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time state_timestamp_;
};

FollowLineNode::FollowLineNode()
    : Node("follow_line_node")
{
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  timer_ = create_wall_timer(
      100ms, std::bind(&FollowLineNode::control_cycle, this));
}

void FollowLineNode::control_cycle()
{

  out_vel_.linear.x = 1.0f;
  out_vel_.angular.z = 1.0f;

  // Publish data
  vel_pub_->publish(out_vel_);
}