import json
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

from gui_interfaces.general.measuring_threading_gui_harmonic import (
    MeasuringThreadingGUI,
)
from console_interfaces.general.console import start_console

# GUI of the myAGV warehouse delivery world
# Everything is drawn on the warehouse map in world coordinates


class DeliveryNode(Node):
    """Robot pose plus the targets and scores of the delivery manager."""

    def __init__(self):
        super().__init__("webgui_delivery_node")
        self.pose = None
        self.values = {}
        self.create_subscription(Odometry, "/myagv_mecharm/odom", self._on_odom, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for color in ("red", "blue"):
            for kind in ("target", "score"):
                key = f"{color}_{kind}"
                self.create_subscription(
                    Int32,
                    f"/warehouse_delivery/{key}",
                    lambda msg, k=key: self.values.__setitem__(k, msg.data),
                    latched,
                )

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = [p.x, p.y, yaw]


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        self.path = []
        self.path_lock = threading.Lock()

        if not rclpy.ok():
            rclpy.init()

        self.delivery_node = DeliveryNode()
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.delivery_node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        self.start()

    def update_gui(self):
        with self.path_lock:
            path = list(self.path)
        state = {
            "pose": self.delivery_node.pose,
            "path": path,
            "red_target": self.delivery_node.values.get("red_target", 0),
            "blue_target": self.delivery_node.values.get("blue_target", 0),
            "red_score": self.delivery_node.values.get("red_score", 0),
            "blue_score": self.delivery_node.values.get("blue_score", 0),
        }
        self.send_to_client(json.dumps({"delivery": json.dumps(state)}))

    def setPath(self, points):
        with self.path_lock:
            self.path = [[float(x), float(y)] for x, y in points]


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)
start_console()


def showPath(points):
    """Draw a path in green on the map.
    points is a list of world (x, y) pairs such as the nodes of a route.
    """
    gui.setPath(points)


def clearPath():
    """Remove the path from the map."""
    gui.setPath([])
