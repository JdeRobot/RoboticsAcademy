from geometry_msgs.msg import Twist
from rclpy.node import Node
import rclpy

if not rclpy.ok():
    rclpy.init()


### HAL INTERFACE ###
class MotorsNode(Node):
    """ROS2 node that publishes linear and angular velocity commands to a robot."""

    def __init__(self, topic, maxV, maxW):

        super().__init__("motors_node")
        self.pub = self.create_publisher(Twist, topic, 10)
        self.last_twist = Twist()

    def sendV(self, v):
        """Publish a linear velocity command.

        Args:
            v (float): Linear velocity in m/s.
        """
        self.last_twist.linear.x = v
        self.pub.publish(self.last_twist)

    def sendW(self, w):
        """Publish an angular velocity command.

        Args:
            w (float): Angular velocity in rad/s.
        """
        self.last_twist.angular.z = w
        self.pub.publish(self.last_twist)
