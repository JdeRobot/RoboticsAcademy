from rclpy.node import Node
from gazebo_msgs.msg import ContactsState

### AUXILIARY FUNCTIONS ###

RIGHT_BUMPER = 0
CENTER_BUMPER = 1
LEFT_BUMPER = 2


class BumperData:

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

    bumper_data = BumperData()

    for i in range(len(contacts)):

        if len(contacts[i].states) > 0:

            bumper_data.state = 1
            bumper_data.bumper = i
            break

    return bumper_data


### HAL INTERFACE ###
class BumperNode(Node):

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

            self.sub = self.create_subscription(
                ContactsState, topics[i], self.callbacks_[i], 10
            )

        # Right, center, left
        self.contact_states_ = [ContactsState() for _ in range(3)]

    def right_callback(self, contact):
        self.contact_states_[0] = contact

    def center_callback(self, contact):
        self.contact_states_[1] = contact

    def left_callback(self, contact):
        self.contact_states_[2] = contact

    def getBumperData(self):
        return contactsToBumperData(self.contact_states_)
