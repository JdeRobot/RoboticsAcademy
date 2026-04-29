import cv2
import base64
import re
import json
import threading
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from sensor_msgs.msg import Image as ROSImage

from hal_interfaces.general.odometry import OdometryNode
from gui_interfaces.general.measuring_threading_gui_harmonic import (
    MeasuringThreadingGUI,
)
from console_interfaces.general.console import start_console
from map import Map


class ROS2BridgeNode(Node):
    def __init__(self, gui_instance):
        super().__init__("gui_bridge_node")
        self.gui = gui_instance

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.target_pub = self.create_publisher(Point, "/webgui/current_target", qos)

        self.create_subscription(Path, "/webgui/path", self.path_callback, 10)
        self.create_subscription(
            ROSImage, "/webgui/debug_image", self.image_callback, 10
        )

    def path_callback(self, msg):
        path_array = []
        for pose_stamped in msg.poses:
            grid_x, grid_y = self.gui.map.worldToGrid(
                pose_stamped.pose.position.x, pose_stamped.pose.position.y
            )
            path_array.append([grid_x, grid_y])
        self.gui.showPath(path_array)

    def image_callback(self, msg):
        try:
            data = np.frombuffer(msg.data, dtype=np.uint8)
            image = data.reshape((msg.height, msg.width))
            self.gui.showNumpy(image)
        except Exception:
            pass

    def publish_target(self, x, y):
        msg = Point(x=float(x), y=float(y), z=0.0)
        self.target_pub.publish(msg)


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)
        self.array = None
        self.array_lock = threading.Lock()
        self.mapXY = None
        self.worldXY = None

        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()

        self.payload = {"image": "", "map": "", "array": ""}

        self.pose3d_node = None
        self.bridge_node = None
        self.executor = None
        self.executor_thread = None

        self._setup_ros2()

        self.map = Map(self.get_pose3d)

        self.start()

    def _setup_ros2(self):
        if not rclpy.ok():
            rclpy.init()

        self.pose3d_node = OdometryNode("/odom")
        self.bridge_node = ROS2BridgeNode(self)

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.pose3d_node)
        self.executor.add_node(self.bridge_node)

        self.executor_thread = threading.Thread(
            target=self.executor.spin, daemon=True, name="webgui_ros2_executor"
        )
        self.executor_thread.start()

    def get_pose3d(self):
        if self.pose3d_node is None:
            return None
        return self.pose3d_node.getPose3d()

    def gui_in_thread(self, ws, message):
        if "ack" in message:
            with self.ack_lock:
                self.ack = True
        elif "start" in message:
            with self.ack_lock:
                self.ack_frontend = True
        elif "pick" in message:
            data = eval(message[4:])
            self.mapXY = data
            x, y = self.mapXY
            self.worldXY = self.map.gridToWorld(x, y)
            if self.bridge_node is not None:
                self.bridge_node.publish_target(self.worldXY[0], self.worldXY[1])

    def update_gui(self):
        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)

        self.payload["array"] = self.array

        pos_message1 = self.map.getTaxiCoordinates()
        ang_message = self.map.getTaxiAngle()
        pos_message = str(pos_message1 + ang_message)
        self.payload["map"] = pos_message

        message = json.dumps(self.payload)
        self.send_to_client(message)

    def payloadImage(self):
        with self.image_show_lock:
            image_to_be_shown_updated = self.image_to_be_shown_updated
            image_to_be_shown = self.image_to_be_shown

        image = image_to_be_shown
        payload = {"image": "", "shape": ""}

        if not image_to_be_shown_updated:
            return payload

        shape = image.shape
        frame = cv2.imencode(".JPEG", image)[1]
        encoded_image = base64.b64encode(frame)

        payload["image"] = encoded_image.decode("utf-8")
        payload["shape"] = shape

        with self.image_show_lock:
            self.image_to_be_shown_updated = False

        return payload

    def showNumpy(self, image):
        processed_image = np.stack((image,) * 3, axis=-1)
        with self.image_show_lock:
            self.image_to_be_shown = processed_image
            self.image_to_be_shown_updated = True

    def showPath(self, array):
        with self.array_lock:
            strArray = "".join(str(e) for e in array)

            strArray = re.sub(r"\[[ ]+", "[", strArray)
            strArray = re.sub(r"[ ]+", ", ", strArray)
            strArray = re.sub(r",[ ]+]", "]", strArray)
            strArray = re.sub(r",,", ",", strArray)
            strArray = re.sub(r"]\[", "],[", strArray)
            strArray = "[" + strArray + "]"

            self.array = strArray

    def getTargetPose(self):
        if self.worldXY is not None:
            return self.worldXY
        else:
            return None

    def getMap(self, url):
        return self.map.getMap(url)

    def worldToGrid(self, pose):
        return self.map.worldToGrid(*pose)

    def gridToWorld(self, cell):
        return self.map.gridToWorld(*cell)

    def reset_gui(self):
        image = [[0 for x in range(400)] for y in range(400)]
        self.showNumpy(np.clip(image, 0, 255).astype("uint8"))
        self.map.reset()

    def __del__(self):
        try:
            if self.executor:
                self.executor.shutdown()
        except Exception:
            pass


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

start_console()


def payloadImage():
    return gui.payloadImage()


def showNumpy(image):
    gui.showNumpy(image)


def showPath(array):
    gui.showPath(array)


def getTargetPose():
    return gui.getTargetPose()


def getMap(url):
    return gui.getMap(url)


def rowColumn(pose):
    return list(gui.worldToGrid(pose))


def worldToGrid(pose):
    return gui.worldToGrid(pose)


def gridToWorld(cell):
    return gui.gridToWorld(cell)
