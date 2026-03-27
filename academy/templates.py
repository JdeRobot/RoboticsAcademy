def select_template(template):
    if template == "py-reactive":
        return python_reactive()
    elif template == "py-ros2":
        return python_ros()
    elif template == "cpp-reactive":
        return cpp_reactive()
    elif template == "cpp-ros2":
        return cpp_ros()
    return ""


def python_reactive():
    return """import WebGUI
import HAL
import Frequency

# Enter sequential code!

while True:
    Frequency.tick()

    # Enter iterative code!
    """


def python_ros():
    return """import rclpy
from rclpy.node import Node
import WebGUI

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        # Enter your code!

def main(args=None):
    # Initialize ROS2 (if not already initialized)
    if not rclpy.ok():
        rclpy.init(args=args)
    
    my_node = MyNode()
    
    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        my_node.get_logger().info("Shutting down...")
    finally:
        my_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
    """


def cpp_reactive():
    return """#include "HAL.hpp"
#include "WebGUI.hpp"
#include "Frequency.hpp"

void exercise() {
    Frequency freq = Frequency();
    // Enter sequential code!

    while (true)
    {
        freq.tick();
        // Enter iterative code!


    }
}
    """


def cpp_ros():
    return """#ifndef USER_NODE
#define USER_NODE

#include "rclcpp/rclcpp.hpp"

class UserNode : public rclcpp::Node {
    // Your class

public:
    UserNode() : Node("user_node")
    {
        // Change to control the frequency
        timer_ = create_wall_timer(100ms, std::bind(&UserNode::control_cycle, this));

        // More subscribers and publishers
    };

    // More Code

    void control_cycle(){
        // Your function
    };

};



#endif

    """
