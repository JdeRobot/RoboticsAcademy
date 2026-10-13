import json
import cv2
import base64
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge

from gui_interfaces.general.measuring_threading_gui_harmonic import (
    MeasuringThreadingGUI,
)
from console_interfaces.general.console import start_console


class ROS2BridgeNode(Node):
    def __init__(self, gui_instance):
        super().__init__("gui_bridge_node_drone")
        self.gui = gui_instance
        self.bridge = CvBridge()

        self.create_subscription(
            RosImage, "/webgui/image_debug_right", self.image_right_callback, 10
        )
        self.create_subscription(
            RosImage, "/webgui/image_debug_left", self.image_left_callback, 10
        )

    def image_right_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.gui.setRightImage(cv_image)

    def image_left_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.gui.setLeftImage(cv_image)


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303", freq=30.0):
        super().__init__(host)
        self.right_image = None
        self.left_image = None
        self.image_lock = threading.Lock()
        self.msg = {"image_right": "", "image_left": ""}

        self.bridge_node = None
        self.executor = None
        self.executor_thread = None

        self._setup_ros2()
        self.start()

    def _setup_ros2(self):
        if not rclpy.ok():
            rclpy.init()
        self.bridge_node = ROS2BridgeNode(self)
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.bridge_node)
        self.executor_thread = threading.Thread(
            target=self.executor.spin, daemon=True, name="webgui_ros2_executor"
        )
        self.executor_thread.start()

    def gui_out_thread(self):
        while self.running:
            start_time = time.time()
            self.iteration_counter += 1

            with self.ack_lock:
                with self.image_lock:
                    if self.ack:
                        if np.any(self.left_image) or np.any(self.right_image):
                            self.update_gui()
                            self.ack = False

            elapsed = time.time() - start_time
            sleep_time = max(0, self.out_period - elapsed)
            time.sleep(sleep_time)

    def update_gui(self):
        if np.any(self.left_image):
            _, encoded_left_image = cv2.imencode(".JPEG", self.left_image)
            b64_left = base64.b64encode(encoded_left_image).decode("utf-8")
            shape_left = self.left_image.shape
        else:
            b64_left = None
            shape_left = 0

        if np.any(self.right_image):
            _, encoded_right_image = cv2.imencode(".JPEG", self.right_image)
            b64_right = base64.b64encode(encoded_right_image).decode("utf-8")
            shape_right = self.right_image.shape
        else:
            b64_right = None
            shape_right = 0

        payload_left = {
            "image_left": b64_left,
            "shape_left": shape_left,
        }
        payload_right = {
            "image_right": b64_right,
            "shape_right": shape_right,
        }
        self.msg["image_left"] = json.dumps(payload_left)
        self.msg["image_right"] = json.dumps(payload_right)
        message = json.dumps(self.msg)
        self.send_to_client(message)

    def setLeftImage(self, image):
        with self.image_lock:
            self.left_image = image

    def setRightImage(self, image):
        with self.image_lock:
            self.right_image = image

    def __del__(self):
        try:
            if self.executor:
                self.executor.shutdown()
        except Exception:
            pass


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

start_console()


def showImage(image):
    gui.setRightImage(image)


def showLeftImage(image):
    gui.setLeftImage(image)
