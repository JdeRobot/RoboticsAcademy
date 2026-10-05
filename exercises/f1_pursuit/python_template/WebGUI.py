import base64
import json
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as RosImage

from console_interfaces.general.console import start_console
from gui_interfaces.general.measuring_threading_gui_harmonic import (
    MeasuringThreadingGUI,
)

CAR_NAMESPACE = "f1"
RIVAL_NAMESPACE = "f1_rival"


class ROS2BridgeNode(Node):
    """Feeds both cockpit cameras into the GUI panels."""

    def __init__(self, gui_instance):
        super().__init__("gui_bridge_node_f1_pursuit")
        self.gui = gui_instance
        self.bridge = CvBridge()

        self.create_subscription(
            RosImage,
            "/" + CAR_NAMESPACE + "/camera/image_raw",
            self.chaser_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            RosImage,
            "/" + RIVAL_NAMESPACE + "/camera/image_raw",
            self.rival_callback,
            qos_profile_sensor_data,
        )

    def chaser_callback(self, msg):
        try:
            self.gui.setLeftImage(
                self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            )
        except Exception:
            pass

    def rival_callback(self, msg):
        try:
            self.gui.setRightImage(
                self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            )
        except Exception:
            pass


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303", freq=30.0):
        super().__init__(host)
        self.left_image = None
        self.right_image = None
        self.image_lock = threading.Lock()
        self.msg = {"image_left": "", "image_right": ""}

        if not rclpy.ok():
            rclpy.init()

        self.bridge_node = ROS2BridgeNode(self)
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.bridge_node)
        self.executor_thread = threading.Thread(
            target=self.executor.spin, daemon=True, name="webgui_ros2_executor"
        )
        self.executor_thread.start()

        self.start()

    def _encode(self, image, key_image, key_shape):
        if image is None or not np.any(image):
            return json.dumps({key_image: None, key_shape: 0})
        _, encoded = cv2.imencode(".JPEG", image)
        return json.dumps(
            {
                key_image: base64.b64encode(encoded).decode("utf-8"),
                key_shape: image.shape,
            }
        )

    def update_gui(self):
        with self.image_lock:
            left, right = self.left_image, self.right_image

        self.msg["image_left"] = self._encode(left, "image_left", "shape_left")
        self.msg["image_right"] = self._encode(right, "image_right", "shape_right")
        self.send_to_client(json.dumps(self.msg))

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


# The panels show the two cameras, there is no free half to draw on
def showImage(image):
    return
