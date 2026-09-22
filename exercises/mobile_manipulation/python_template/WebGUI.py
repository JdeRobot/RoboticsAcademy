import base64
import json
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from gui_interfaces.general.measuring_threading_gui_harmonic import (
    MeasuringThreadingGUI,
)
from console_interfaces.general.console import start_console
from hal_interfaces.general.camera import CameraNode

# showImage feeds the color panel and showDepthImage feeds the grayscale depth panel
# The /webgui_image fallback only feeds the color panel

MAX_DEPTH_METERS = 4.0


class WebGUIImagePublisher(Node):
    """Internal publisher, lets a separate-process solution reach this GUI too."""

    def __init__(self):
        super().__init__("webgui_image_publisher_internal")
        self.publisher = self.create_publisher(Image, "/webgui_image", 10)


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        self.right_image = None
        self.left_image = None
        self.image_lock = threading.Lock()
        self.msg = {"image_right": "", "image_left": ""}

        if not rclpy.ok():
            rclpy.init()

        self.webgui_publisher = WebGUIImagePublisher()
        self.camera_node = None
        self.auto_image_mode = False
        self._setup_auto_mode()

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.webgui_publisher)
        if self.camera_node:
            self.executor.add_node(self.camera_node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        if self.auto_image_mode:
            self.auto_image_thread = threading.Thread(
                target=self._unified_image_loop, daemon=True
            )
            self.auto_image_thread.start()

        self.start()

    def _setup_auto_mode(self):
        """If a separate process is already publishing /webgui_image, follow it."""
        try:
            temp_node = rclpy.create_node("topic_checker_temp")
            topic_names_and_types = temp_node.get_topic_names_and_types()
            topic_names = [topic_name for topic_name, _ in topic_names_and_types]

            if "/webgui_image" in topic_names:
                self.camera_node = CameraNode("/webgui_image")
                self.auto_image_mode = True

            temp_node.destroy_node()

        except Exception:
            pass

    def _unified_image_loop(self):
        while True:
            try:
                if self.camera_node:
                    image = self.camera_node.getImage()
                    if image is not None:
                        self.setRightImage(image.data)

                threading.Event().wait(0.033)  # ~30 FPS
            except Exception:
                threading.Event().wait(1.0)

    def update_gui(self):
        with self.image_lock:
            right_image = self.right_image
            left_image = self.left_image

        payload_right = {"image_right": "", "shape_right": 0}
        if np.any(right_image):
            _, encoded = cv2.imencode(".JPEG", right_image)
            payload_right["image_right"] = base64.b64encode(encoded).decode("utf-8")
            payload_right["shape_right"] = right_image.shape

        payload_left = {"image_left": "", "shape_left": 0}
        if np.any(left_image):
            _, encoded = cv2.imencode(".JPEG", left_image)
            payload_left["image_left"] = base64.b64encode(encoded).decode("utf-8")
            payload_left["shape_left"] = left_image.shape

        self.msg["image_right"] = json.dumps(payload_right)
        self.msg["image_left"] = json.dumps(payload_left)
        self.send_to_client(json.dumps(self.msg))

    def setRightImage(self, image):
        with self.image_lock:
            self.right_image = image

    def setLeftImage(self, image):
        with self.image_lock:
            self.left_image = image


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)
start_console()


def showImage(image):
    """Display a BGR numpy image in the right-hand panel, call this from
    the solution with HAL.getImage()."""
    gui.setRightImage(image)


def showDepthImage(depth):
    """Display a raw HAL.getDepthImage() array (float32 meters, may contain
    inf/nan for out-of-range pixels) in the left-hand panel as a viewable
    grayscale image: closer is brighter, invalid/out-of-range is black.
    Values beyond MAX_DEPTH_METERS are clipped, not stretched to it, so a
    stray far reading does not wash out the whole scale.
    """
    finite = np.where(np.isfinite(depth), depth, MAX_DEPTH_METERS)
    clipped = np.clip(finite, 0.0, MAX_DEPTH_METERS)
    gray = ((1.0 - clipped / MAX_DEPTH_METERS) * 255.0).astype(np.uint8)
    gui.setLeftImage(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR))
