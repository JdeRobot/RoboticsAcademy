from geometry_msgs.msg import Twist
from rclpy.node import Node


### HAL INTERFACE ###
class MotorsNode(Node):

    def __init__(self, topic, maxV, maxW):

        super().__init__("motors_node")
        self.pub = self.create_publisher(Twist, topic, 10)
        self.last_twist = Twist()

    def sendV(self, v):
        self.last_twist.linear.x = v
        self.pub.publish(self.last_twist)

    def sendW(self, w):
        self.last_twist.angular.z = w
        self.pub.publish(self.last_twist)
