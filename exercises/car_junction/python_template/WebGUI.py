import json
import cv2
import base64
import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.executors import SingleThreadedExecutor

from gui_interfaces.general.measuring_threading_gui_harmonic import MeasuringThreadingGUI
from console_interfaces.general.console import start_console


class WebGUINode(Node):
    def __init__(self, gui_instance):
        super().__init__("webgui_car_junction")
        self.gui = gui_instance
        self.bridge = CvBridge()
        
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Image, '/webgui/image', self.img_cb, qos_profile)

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

        self.payload = {"image": ""}

        self.ros_node = WebGUINode(self)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.ros_node)

        self.start()

    def payloadImage(self):
        with self.image_show_lock:
            image_to_be_shown_updated = self.image_to_be_shown_updated
            image_to_be_shown = self.image_to_be_shown

        payload = {"image": "", "shape": ""}

        if not image_to_be_shown_updated or image_to_be_shown is None:
            return payload

        shape = image_to_be_shown.shape
        frame = cv2.imencode(".JPEG", image_to_be_shown)[1]
        encoded_image = base64.b64encode(frame)

        payload["image"] = encoded_image.decode("utf-8")
        payload["shape"] = shape

        with self.image_show_lock:
            self.image_to_be_shown_updated = False

        return payload

    def update_gui(self):
        self.executor.spin_once(timeout_sec=0)

        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)

        message = json.dumps(self.payload)
        self.send_to_client(message)

    def setImage(self, image):
        with self.image_show_lock:
            self.image_to_be_shown = image
            self.image_to_be_shown_updated = True


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)
start_console()

def showImage(image):
    gui.setImage(image)