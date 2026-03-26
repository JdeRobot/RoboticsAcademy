import json
import subprocess
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from gui_interfaces.general.measuring_threading_gui_harmonic import (
    MeasuringThreadingGUI,
)
from console_interfaces.general.console import start_console
from map import Map


class LaserData:
    def __init__(self, maxAngle, maxRange, values):
        self.maxAngle = maxAngle
        self.maxRange = maxRange
        self.values = values


class WebGUINode(Node):
    def __init__(self):
        super().__init__("autoparking_gui_node")

        self.sub_scan_front = self.create_subscription(
            LaserScan, "/prius_autoparking/scan_front", self.scan_f_cb, 10
        )
        self.sub_scan_side = self.create_subscription(
            LaserScan, "/prius_autoparking/scan_side", self.scan_r_cb, 10
        )
        self.sub_scan_back = self.create_subscription(
            LaserScan, "/prius_autoparking/scan_back", self.scan_b_cb, 10
        )
        self.sub_pc2 = self.create_subscription(
            PointCloud2, "/prius_autoparking/pc2", self.pc2_cb, 10
        )

        self.laser_f = None
        self.laser_r = None
        self.laser_b = None
        self.lidar = None

    def _parse_laser(self, msg):
        fov = msg.angle_max - msg.angle_min
        return LaserData(fov, msg.range_max, msg.ranges)

    def scan_f_cb(self, msg):
        self.laser_f = self._parse_laser(msg)

    def scan_r_cb(self, msg):
        self.laser_r = self._parse_laser(msg)

    def scan_b_cb(self, msg):
        self.laser_b = self._parse_laser(msg)

    def pc2_cb(self, msg):
        self.lidar = msg


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        if not rclpy.ok():
            rclpy.init()

        self.ros_node = WebGUINode()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.ros_node)

        self.payload = {"map": ""}
        self.map = Map(self.get_front, self.get_right, self.get_back)

        self.mode = "Laser"

        args = ["ros2", "topic", "info", "/prius_autoparking/pc2"]
        topic_info = subprocess.Popen(args, stdout=subprocess.PIPE)
        for line in topic_info.stdout:
            if line == b"Publisher count: 1\n":
                self.mode = "Lidar"

        self.start()

    def get_front(self):
        return self.ros_node.laser_f

    def get_right(self):
        return self.ros_node.laser_r

    def get_back(self):
        return self.ros_node.laser_b

    def update_gui(self):
        self.executor.spin_once(timeout_sec=0)

        if self.mode == "Laser":
            if (
                self.ros_node.laser_f is not None
                and self.ros_node.laser_r is not None
                and self.ros_node.laser_b is not None
            ):
                map_message = self.map.get_json_data()
                self.payload["map"] = map_message

        elif self.mode == "Lidar":
            if self.ros_node.lidar:
                points = []
                color = (20, 20, 255)
                for p in pc2.read_points(
                    self.ros_node.lidar, field_names=("x", "y", "z"), skip_nans=True
                ):
                    x, y, z = p
                    if not (math.isinf(x) or math.isinf(y) or math.isinf(z)):
                        points.append(
                            (float(x * 10), float((z + 1.75) * 10), float(-y * 10))
                            + color
                        )
                self.payload["lidar"] = json.dumps(points)

        message = json.dumps(self.payload)
        self.send_to_client(message)

    def reset_gui(self):
        self.map = Map(self.get_front, self.get_right, self.get_back)


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)
start_console()
