from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile, HistoryPolicy
import rosgraph_msgs.msg

### AUXILIARY FUNCTIONS
class SimTimeData:

    def __init__(self):

        self.seconds = 0
        self.nanoseconds = 0

    def __str__(self):
        s = (
            "SimTimeData: {\n   sec: "
            + str(self.seconds)
            + "\n   nanosec: "
            + str(self.nanoseconds)
        )
        return s

def simTime2SimTimeData(clock):
    """
    Translates from ROS Clock to JderobotTypes SimTimeData.
    @param clock: ROS Clock to translate
    @type clock: Clock
    @return a SimTimeData translated from clock
    """
    clockData = SimTimeData()
    clockData.seconds = clock.clock.sec
    clockData.nanoseconds = clock.clock.nanosec
    return clockData

### HAL INTERFACE ###
class SimTimeNode(Node):

    def __init__(self):
        super().__init__("simulation_time_node")
        qos_policy = QoSProfile(reliability= ReliabilityPolicy.BEST_EFFORT, history= HistoryPolicy.KEEP_LAST, depth=1)
        self.sub = self.create_subscription(
            rosgraph_msgs.msg.Clock, "/clock", self.listener_callback, qos_profile=qos_policy
        )
        self.last_sim_time_ = rosgraph_msgs.msg.Clock()

    def listener_callback(self, msg):
        self.last_sim_time_ = msg

    def getSimTime(self):
        return simTime2SimTimeData(self.last_sim_time_)
