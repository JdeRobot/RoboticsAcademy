import base64
import json
import threading
import sys
import math
import cv2
import re
import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image as ROSImage
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String

from gui_interfaces.general.measuring_threading_gui_harmonic import (
    MeasuringThreadingGUI,
)
from map import Map
from console_interfaces.general.console import start_console

red = [0, 0, 255]
orange = [0, 165, 255]
yellow = [0, 255, 255]
green = [0, 255, 0]
blue = [255, 0, 0]
indigo = [130, 0, 75]
violet = [211, 0, 148]


def quat_to_yaw(qw, qx, qy, qz):
    rotate_za0 = 2.0 * (qx * qy + qw * qz)
    rotate_za1 = qw * qw + qx * qx - qy * qy - qz * qz
    if rotate_za0 != 0.0 and rotate_za1 != 0.0:
        return math.atan2(rotate_za0, rotate_za1)
    return 0.0


class Pose3d:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw


class GUIBridgeNode(Node):
    def __init__(self, gui_instance):
        super().__init__("gui_bridge_node")
        self.gui = gui_instance
        self.pose = Pose3d()
        self.lift_state = False

        qos_transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(
            Odometry, "/amazon_robot/odom", self.odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(Float64, "/platform/cmd_vel", self.lift_callback, 10)
        self.create_subscription(
            String, "/webgui/path", self.path_callback, qos_transient
        )
        self.create_subscription(
            ROSImage, "/webgui/image", self.image_callback, qos_transient
        )

    def odom_callback(self, msg):
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        ori = msg.pose.pose.orientation
        self.pose.yaw = quat_to_yaw(ori.w, ori.x, ori.y, ori.z)

    def lift_callback(self, msg):
        if msg.data > 0.0:
            self.lift_state = True
        elif msg.data < 0.0:
            self.lift_state = False

    def path_callback(self, msg):
        self.gui.update_path_array(msg.data)

    def image_callback(self, msg):
        channels = 1 if msg.encoding in ["mono8", "8UC1"] else 3
        shape = (
            (msg.height, msg.width)
            if channels == 1
            else (msg.height, msg.width, channels)
        )
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(shape)
        self.gui.showNumpy(image)


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        self.array_lock = threading.Lock()
        self.array = ""

        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()

        self.payload = {"map": "", "array": "", "liftState": False}

        self.bridge_node = None
        self.executor = None
        self.executor_thread = None

        self._setup_ros2()

        self.map = Map(self.get_pose3d)

        self.init_coords = self.map.getRobotCoordinates()
        self.start_coords = self.map.getRobotCoordinates()

        self.start()

    def _setup_ros2(self):
        if not rclpy.ok():
            rclpy.init(args=sys.argv)

        self.bridge_node = GUIBridgeNode(self)
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.bridge_node)

        self.executor_thread = threading.Thread(
            target=self.executor.spin, daemon=True, name="webgui_ros2_executor"
        )
        self.executor_thread.start()

    def get_pose3d(self):
        return self.bridge_node.pose

    def get_lift_state(self):
        return self.bridge_node.lift_state

    def update_gui(self):
        with self.array_lock:
            self.payload["array"] = self.array

        self.payload["liftState"] = self.get_lift_state()

        pos_message = self.map.getRobotCoordinates()
        ang_message = self.map.getRobotAngle()
        self.payload["map"] = str(pos_message + ang_message)

        payload_img = self.payloadImage()
        self.payload["image"] = json.dumps(payload_img)

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

    def process_colors(self, image):
        colored_image = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)

        mask = image < 128
        colored_image[mask] = image[mask][:, None] * 2

        color_table = {
            128: red,
            129: orange,
            130: yellow,
            131: green,
            132: blue,
            133: indigo,
            134: violet,
        }

        for value, color in color_table.items():
            mask = image == value
            colored_image[mask] = color

        return colored_image

    def showNumpy(self, image):
        with self.image_show_lock:
            self.image_to_be_shown = self.process_colors(image)
            self.image_to_be_shown_updated = True

    def update_path_array(self, strArray):
        with self.array_lock:
            self.array = strArray

    def showPath(self, array):
        array_scaled = []
        for wp in array:
            array_scaled.append([wp[0] * 0.72, wp[1] * 0.545])

        strArray = "".join(str(e) for e in array_scaled)
        strArray = re.sub(r"\[[ ]+", "[", strArray)
        strArray = re.sub(r"[ ]+", ", ", strArray)
        strArray = re.sub(r",[ ]+]", "]", strArray)
        strArray = re.sub(r",,", ",", strArray)
        strArray = re.sub(r"]\[", "],[", strArray)
        strArray = "[" + strArray + "]"

        with self.array_lock:
            self.array = strArray

    def getMap(self, url):
        return plt.imread(url)

    def __del__(self):
        try:
            if self.executor:
                self.executor.shutdown()
        except Exception:
            pass


if not rclpy.ok():
    rclpy.init(args=sys.argv)

host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

start_console()


def getMap(url):
    return gui.getMap(url)


def showPath(array):
    return gui.showPath(array)


def showNumpy(image):
    gui.showNumpy(image)
