import json
import threading
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy

from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.laser import LaserNode
from gui_interfaces.general.measuring_threading_gui_harmonic import (
    MeasuringThreadingGUI,
)
from console_interfaces.general.console import start_console
from lap import Lap
from map import Map


class ROS2BridgeNode(Node):
    def __init__(self, gui_instance):
        super().__init__("gui_bridge_node")
        self.gui = gui_instance

        self.create_subscription(
            Point, "/webgui/force/car", self.force_car_callback, 10
        )
        self.create_subscription(
            Point, "/webgui/force/obs", self.force_obs_callback, 10
        )
        self.create_subscription(
            Point, "/webgui/force/avg", self.force_avg_callback, 10
        )
        self.create_subscription(
            Point, "/webgui/local_target", self.target_callback, 10
        )
        self.create_subscription(
            Bool, "/webgui/target_reached", self.target_reached_callback, 10
        )

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.current_target_pub = self.create_publisher(
            Point, "/webgui/current_target", qos
        )

        self.current_target_obj = self.gui.map.getNextTarget()
        self.publish_current_target()

    def force_car_callback(self, msg):
        self.gui.map.setCar(msg.x, msg.y)

    def force_obs_callback(self, msg):
        self.gui.map.setObs(msg.x, msg.y)

    def force_avg_callback(self, msg):
        self.gui.map.setAvg(msg.x, msg.y)

    def target_callback(self, msg):
        self.gui.map.setTargetPos(msg.x, msg.y)

    def target_reached_callback(self, msg):
        if msg.data and self.current_target_obj:
            self.current_target_obj.setReached(True)
            self.current_target_obj = self.gui.map.getNextTarget()
            self.publish_current_target()

    def publish_current_target(self):
        if self.current_target_obj:
            msg = Point(
                x=float(self.current_target_obj.getPose().x),
                y=float(self.current_target_obj.getPose().y),
                z=0.0,
            )
            self.current_target_pub.publish(msg)


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        self.payload = {"lap": "", "map": ""}

        self.pose3d_node = None
        self.laser_node = None
        self.bridge_node = None
        self.executor = None
        self.executor_thread = None

        self._setup_ros2()
        self.start()

    def _setup_ros2(self):
        if not rclpy.ok():
            rclpy.init()

        self.pose3d_node = OdometryNode("/odom")
        self.laser_node = LaserNode("/f1/laser/scan")

        self.map = Map(self.get_laser_data, self.get_pose3d)
        self.lap = Lap(self.map)

        self.bridge_node = ROS2BridgeNode(self)

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.pose3d_node)
        self.executor.add_node(self.laser_node)
        self.executor.add_node(self.bridge_node)

        self.executor_thread = threading.Thread(
            target=self.executor.spin, daemon=True, name="webgui_ros2_executor"
        )
        self.executor_thread.start()

    def get_laser_data(self):
        if self.laser_node is None:
            return None
        laser_data = self.laser_node.getLaserData()
        while laser_data is not None and len(laser_data.values) == 0:
            laser_data = self.laser_node.getLaserData()
        return laser_data

    def get_pose3d(self):
        if self.pose3d_node is None:
            return None
        return self.pose3d_node.getPose3d()

    def update_gui(self):
        lapped = self.lap.check_threshold()
        if lapped is not None:
            self.payload["lap"] = str(lapped)

        map_message = self.map.get_json_data()
        self.payload["map"] = map_message

        message = json.dumps(self.payload)
        self.send_to_client(message)

    def showForces(self, vec1, vec2, vec3):
        self.map.setCar(vec1[0], vec1[1])
        self.map.setObs(vec2[0], vec2[1])
        self.map.setAvg(vec3[0], vec3[1])

    def showLocalTarget(self, newVec):
        self.map.setTargetPos(newVec[0], newVec[1])

    def reset_gui(self):
        self.map.reset()
        self.lap.reset()

    def __del__(self):
        try:
            if self.executor:
                self.executor.shutdown()
        except Exception:
            pass


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

start_console()


def showForces(vec1, vec2, vec3):
    gui.showForces(vec1, vec2, vec3)


def showLocalTarget(newVec):
    return gui.showLocalTarget(newVec)


def getNextTarget():
    return gui.map.getNextTarget()


def setTargetx(x):
    gui.map.targetx = x


def setTargety(y):
    gui.map.targety = y
