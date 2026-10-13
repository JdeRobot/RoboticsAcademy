import json
import cv2
import base64
import threading
import time
import numpy as np

import rclpy
from gui_interfaces.general.measuring_threading_gui_harmonic import (
    MeasuringThreadingGUI,
)
from console_interfaces.general.console import start_console
from hal_interfaces.general.camera import CameraNode
from sensor_msgs.msg import Image
from rclpy.node import Node

# Graphical User Interface Class


class WebGUIImagePublisher(Node):
    """Internal publisher to create /webgui_image topic"""

    def __init__(self):
        super().__init__("webgui_image_publisher_internal")
        self.publisher = self.create_publisher(Image, "/webgui_image", 10)


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303", freq=30.0):
        super().__init__(host)

        # Execution control vars
        self.right_image = None
        self.image_lock = threading.Lock()
        self.msg = {"image_right": ""}

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
        """Set up automatic image subscription"""
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
        """Unified image handling loop"""
        while True:
            try:
                if self.camera_node:
                    image = self.camera_node.getImage()
                    if image is not None:
                        self.setRightImage(image.data)

                threading.Event().wait(0.033)  # ~30 FPS
            except Exception:
                threading.Event().wait(1.0)

    # Process outcoming messages from the GUI
    def gui_out_thread(self):
        while self.running:
            start_time = time.time()
            self.iteration_counter += 1

            # Check if a new image should be sent
            with self.ack_lock:
                with self.image_lock:
                    if self.ack:
                        if np.any(self.right_image):
                            self.update_gui()
                            self.ack = False

            # Maintain desired frequency
            elapsed = time.time() - start_time
            sleep_time = max(0, self.out_period - elapsed)
            time.sleep(sleep_time)

    # Prepares and send image to the websocket server
    def update_gui(self):

        if np.any(self.right_image):
            _, encoded_right_image = cv2.imencode(".JPEG", self.right_image)
            b64_right = base64.b64encode(encoded_right_image).decode("utf-8")
            shape_right = self.right_image.shape
        else:
            b64_right = None
            shape_right = 0

        payload_right = {
            "image_right": b64_right,
            "shape_right": shape_right,
        }

        self.msg["image_right"] = json.dumps(payload_right)
        message = json.dumps(self.msg)
        self.send_to_client(message)

    # Functions to set the next image to be sent
    def setRightImage(self, image):
        with self.image_lock:
            self.right_image = image

    def get_image_mode(self):
        return {
            "auto_mode": self.auto_image_mode,
            "topic_subscribed": "/webgui_image" if self.auto_image_mode else None,
            "manual_mode_available": True,
        }


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

# Redirect the console
start_console()


# Expose the user functions
def showImage(image):
    gui.setRightImage(image)


def get_image_mode():
    if gui is not None:
        return gui.get_image_mode()
    return {"auto_mode": False, "topic_subscribed": None, "manual_mode_available": True}
