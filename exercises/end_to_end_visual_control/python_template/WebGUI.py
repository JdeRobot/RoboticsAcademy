import cv2
import base64
import json
import threading
import sys
import rclpy
from rclpy.executors import MultiThreadedExecutor
from gui_interfaces.general.measuring_threading_gui_harmonic import (
    MeasuringThreadingGUI,
)
from console_interfaces.general.console import start_console
from lap import Lap
from cv_bridge import CvBridge
from hal_interfaces.general.odometry import OdometryNode
from sensor_msgs.msg import Image
from rclpy.node import Node


class WebGUINode(Node):
    def __init__(self, gui_instance):
        super().__init__("webgui_end_to_end")
        self.gui = gui_instance
        self.bridge = CvBridge()

        self.create_subscription(Image, "/webgui/image", self.img_cb, 10)

    def img_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.gui.image_show_lock:
                self.gui.image_to_be_shown = cv_image
                self.gui.image_to_be_shown_updated = True
        except Exception:
            pass


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        if not rclpy.ok():
            rclpy.init()

        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()

        self.payload = {"image": "", "lap": "", "map": ""}

        self.ros_node = WebGUINode(self)
        self.odom_node = OdometryNode("/f1/odom", "webgui_odometry")

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.ros_node)
        self.executor.add_node(self.odom_node)

        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        self.lap = Lap(self.odom_node)

        self.start()

    def update_gui(self):
        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)

        lapped = self.lap.check_threshold()
        self.payload["lap"] = ""
        if lapped is not None:
            self.payload["lap"] = str(lapped)

        pose = self.odom_node.getPose3d()
        pos_message = str((pose.x, pose.y))
        self.payload["map"] = pos_message

        message = json.dumps(self.payload)
        self.send_to_client(message)

    def payloadImage(self):
        with self.image_show_lock:
            image_to_be_shown_updated = self.image_to_be_shown_updated
            image_to_be_shown = self.image_to_be_shown

        payload = {"image": "", "shape": ""}

        if not image_to_be_shown_updated:
            return payload

        if image_to_be_shown is not None:
            shape = image_to_be_shown.shape
            frame = cv2.imencode(".JPEG", image_to_be_shown)[1]
            encoded_image = base64.b64encode(frame)

            payload["image"] = encoded_image.decode("utf-8")
            payload["shape"] = shape

        with self.image_show_lock:
            self.image_to_be_shown_updated = False

        return payload

    def showImage(self, image):
        with self.image_show_lock:
            self.image_to_be_shown = image
            self.image_to_be_shown_updated = True


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)
start_console()


def showImage(image):
    gui.showImage(image)
