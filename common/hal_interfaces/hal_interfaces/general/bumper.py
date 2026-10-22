from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
import rclpy

if not rclpy.ok():
    rclpy.init()

### AUXILIARY FUNCTIONS ###

RIGHT_BUMPER = 0
CENTER_BUMPER = 1
LEFT_BUMPER = 2


class BumperData:
    """Bumper contact state: whether a bumper is pressed and which one."""
    def __init__(self):

        self.state = 0
        self.bumper = CENTER_BUMPER

    def __str__(self):
        s = (
            "Bumper: {\n   state: "
            + str(self.state)
            + "\n   bumper: "
            + str(self.bumper)
            + "\n}"
        )
        return s


def contactsToBumperData(contacts):
    """
    Convert per-bumper Gazebo contact messages to a BumperData object.

    Args:
        contacts: List of ros_gz_interfaces/Contacts messages ordered
            right, center, left.

    Returns:
        BumperData with state 1 and the index of the first bumper in contact,
        or state 0 if none of them is touching anything.
    """

    bumper_data = BumperData()

    for i in range(len(contacts)):
        if len(contacts[i].contacts) > 0:
            bumper_data.state = 1
            bumper_data.bumper = i
            break

    return bumper_data


### HAL INTERFACE ###
class BumperNode(Node):
    """ROS2 node that subscribes to the right, center and left bumper topics."""
    def __init__(self, topics):
        super().__init__("bumper_node")

        # Hardcoded for the moment to three topics
        # as dynamic callback creation is not trivial
        self.topics = topics
        self.callbacks_ = [
            self.right_callback,
            self.center_callback,
            self.left_callback,
        ]

        # Subscribe to all the callbacks
        for i in range(len(self.topics)):
            self.create_subscription(Contacts, topics[i], self.callbacks_[i], 10)

        # Right, center, left
        self.contact_states_ = [Contacts() for _ in range(3)]

    def right_callback(self, contact):
        """Store the latest contacts from the right bumper."""
        self.contact_states_[0] = contact

    def center_callback(self, contact):
        """Store the latest contacts from the center bumper."""
        self.contact_states_[1] = contact

    def left_callback(self, contact):
        """Store the latest contacts from the left bumper."""
        self.contact_states_[2] = contact

    def getBumperData(self):
        """
        Return the current bumper state.

        Returns:
            BumperData built from the latest contacts of the three bumpers.
        """
        return contactsToBumperData(self.contact_states_)
