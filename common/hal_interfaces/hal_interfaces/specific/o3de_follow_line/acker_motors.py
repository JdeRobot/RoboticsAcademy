import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from math import atan

if not rclpy.ok():
    rclpy.init()

class AckerMotorsNode(Node):
    def __init__ (self, topic, maxV, maxW, L):
        super().__init__("ackermotors_node")
        self.pub=self.create_publisher(AckermannDrive, topic, 10)
        self.last_acker=AckermannDrive()
        self.L=L

    def setV(self, v):
        self.last_acker.speed=v
        self.pub.publish(self.last_acker)

    def setW(self, w):
        if self.last_acker.speed == 0 or w==0:
            self.last_acker.steering_angle = 0

        velocity=abs(self.last_acker.speed)

        self.last_acker.steering_angle=atan(self.L * w / velocity)
        
