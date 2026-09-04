from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts, Contact
import sensor_msgs.msg
import rclpy

if not rclpy.ok():
    rclpy.init()


### HAL INTERFACE ###
class LaserBumperNode(Node):
    def __init__(self, laser, topics):
        super().__init__("laser_bumper_node")

        # Using laser instead of bumper
        self.sub = self.create_subscription(
            sensor_msgs.msg.LaserScan, laser, self.listener_callback, 10
        )
        self.last_scan_ = sensor_msgs.msg.LaserScan()

        self.topics = topics
        self.publishers_ = []

        # Create all publishers
        for i in range(len(self.topics)):
            self.publishers_.append(self.create_publisher(Contacts, topics[i], 10))

    def listener_callback(self, scan):

        values = scan.ranges
        bumper_vals = [values[60], values[90], values[120]]

        contact = Contact()
        msg = Contacts()
        msg.contacts = [contact]

        for i in range(len(bumper_vals)):
            if bumper_vals[i] < 0.5:
                self.publishers_[i].publish(msg)
