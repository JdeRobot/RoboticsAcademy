from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts, Contact
import sensor_msgs.msg
import rclpy

if not rclpy.ok():
    rclpy.init()


### HAL INTERFACE ###
class LaserBumperNode(Node):
    """
    ROS2 node that emulates bumpers using a laser scanner.

    Reads a LaserScan and publishes Contacts messages on the bumper topics
    so robots without bumper sensors can still use BumperNode.
    """
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

        self.contact_states_ = [Contacts() for _ in range(3)]

    def listener_callback(self, scan):
        """
        Publish bumper contacts from the latest laser scan.

        The ranges at indices 60, 90 and 120 act as the right, center and left
        bumpers. A reading below 0.25 m publishes a contact on that bumper's
        topic; once it clears, an empty Contacts message is published once to
        release it.
        """

        values = scan.ranges
        bumper_vals = [values[60], values[90], values[120]]

        contact = Contact()
        msg = Contacts()
        msg.contacts = [contact]

        for i in range(len(bumper_vals)):
            if bumper_vals[i] < 0.25:
                self.publishers_[i].publish(msg)
                self.contact_states_[i] = True
            elif self.contact_states_[i]:
                self.publishers_[i].publish(Contacts())
                self.contact_states_[i] = False
