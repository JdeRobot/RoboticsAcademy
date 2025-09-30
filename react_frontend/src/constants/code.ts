export const defaultPythonCode = `import WebGUI
import HAL
import Frequency
# Enter sequential code!

while True:
    # Enter iterative code!
    Frequency.tick()
`;

export const defaultCppCode = `#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "HAL.hpp"
#include "opencv2/opencv.hpp"

using namespace std::chrono_literals; // NOLINT
using std::placeholders::_1;

class FollowLineNode : public rclcpp::Node
{
public:
  FollowLineNode();

private:
  // Control cycle
  void control_cycle();

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time state_timestamp_;
};

FollowLineNode::FollowLineNode()
    : Node("follow_line_node")
{

  timer_ = create_wall_timer(
      100ms, std::bind(&FollowLineNode::control_cycle, this));
}

void FollowLineNode::control_cycle()
{
  cv::Mat img = HAL::get_image();
  HAL::set_v(1.0f);
  HAL::set_w(1.0f);
}


`;