import base64
import json
import threading
import traceback

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from console_interfaces.general.console import start_console
from gui_interfaces.general.measuring_threading_gui_harmonic import (
    MeasuringThreadingGUI,
)


class WebGUINode(Node):
    """Debug images sent over ROS 2."""

    def __init__(self, gui):
        super().__init__("webgui_tomatoes_harvesting")
        self.gui = gui
        self.bridge = CvBridge()
        self.create_subscription(Image, "/webgui/image_debug_left", self.on_left_image, 10)
        self.create_subscription(Image, "/webgui/image_debug_right", self.on_right_image, 10)

    def on_left_image(self, msg):
        self.gui.showLeftImage(self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8"))

    def on_right_image(self, msg):
        self.gui.showImage(self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8"))


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        self.lock = threading.Lock()
        self.images = {"left": None, "right": None}

        if not rclpy.ok():
            rclpy.init()
        self.node = WebGUINode(self)
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        # Own name so the HAL, which mutes errors of threads called spin, never hides a failure here
        self.executor_thread = threading.Thread(target=self._spin, name="webgui_ros", daemon=True)
        self.executor_thread.start()

        self.start()

    def _spin(self):
        try:
            self.executor.spin()
        except Exception:
            traceback.print_exc()

    def update_gui(self):
        with self.lock:
            images, self.images = self.images, {"left": None, "right": None}
        payload = {
            "image_left": encode_image(images["left"]),
            "image_right": encode_image(images["right"]),
        }
        self.send_to_client(json.dumps(payload))

    def showImage(self, image):
        with self.lock:
            self.images["right"] = image

    def showLeftImage(self, image):
        with self.lock:
            self.images["left"] = image


def encode_image(image):
    # Only new images are sent, the frontend keeps showing the last one meanwhile
    if image is None:
        return ""
    frame = cv2.imencode(".JPEG", image)[1]
    return json.dumps({"image": base64.b64encode(frame).decode("utf-8"), "shape": image.shape})


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

start_console()


def showImage(image):
    """Show an image in the right panel of the GUI."""
    gui.showImage(image)


def showLeftImage(image):
    """Show an image in the left panel of the GUI."""
    gui.showLeftImage(image)
