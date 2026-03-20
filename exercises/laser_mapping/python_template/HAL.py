import rclpy
import sys
import threading
import time
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, DurabilityPolicy

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.laser import LaserNode

freq = 90.0

def custom_thread_excepthook(args):
    if "spin" in args.thread.name:
        return
    sys.__excepthook__(args.exc_type, args.exc_value, args.exc_traceback)

threading.excepthook = custom_thread_excepthook

def __auto_spin() -> None:
    while rclpy.ok():
        try:
            executor.spin_once(timeout_sec=0)
        except Exception:
            pass
        time.sleep(1 / freq)

if not rclpy.ok():
    rclpy.init(args=sys.argv)

class OdomSelectorNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('hal_odom_selector')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Int32, "/webgui/selected_odom", qos)

    def select(self, level):
        msg = Int32()
        msg.data = int(level)
        self.pub.publish(msg)

motor_node = MotorsNode("/turtlebot3/cmd_vel", 4, 0.3)
odometry_node = OdometryNode("/turtlebot3/odom")
noisy_odometry_node_1 = OdometryNode("/turtlebot3/odom_noisy_1")
noisy_odometry_node_2 = OdometryNode("/turtlebot3/odom_noisy_2")
noisy_odometry_node_3 = OdometryNode("/turtlebot3/odom_noisy_3")
laser_node = LaserNode("/turtlebot3/laser/scan")
selector_node = OdomSelectorNode()

executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(odometry_node)
executor.add_node(noisy_odometry_node_1)
executor.add_node(noisy_odometry_node_2)
executor.add_node(noisy_odometry_node_3)
executor.add_node(laser_node)
executor.add_node(selector_node)

executor_thread = threading.Thread(target=__auto_spin, daemon=True)
executor_thread.start()

def getPose3d():
    selector_node.select(0)
    return odometry_node.getPose3d()

def getOdom():
    selector_node.select(1)
    return noisy_odometry_node_1.getPose3d()

def getOdom2():
    selector_node.select(2)
    return noisy_odometry_node_2.getPose3d()

def getOdom3():
    selector_node.select(3)
    return noisy_odometry_node_3.getPose3d()

def getLaserData():
    laser_data = laser_node.getLaserData()
    while len(laser_data.values) == 0:
        laser_data = laser_node.getLaserData()
    return laser_data

def setV(v):
    motor_node.sendV(float(v))

def setW(w):
    motor_node.sendW(float(w))