"""
WebGUI for Basic Computer Vision Exercise
Supports dual mode: Python API (getImage/showImage) and ROS2 topics
Now extended with Visual Odometry channel (odom)
"""

import json
import cv2
import base64
import threading
import rclpy
import sys

from gui_interfaces.general.measuring_threading_gui_no_sim import MeasuringThreadingGUI
from console_interfaces.general.console import start_console
from hal_interfaces.general.camera import CameraNode
import numpy as np

import HAL


class WebGUI(MeasuringThreadingGUI):

    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        # -----------------------------
        # IMAGE STATE
        # -----------------------------
        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()

        self.frame_rgb = None
        self.frame_rgb_lock = threading.Lock()

        # -----------------------------
        # PAYLOAD (IMAGE + ODOM)
        # -----------------------------
        self.payload = {
            "image": "",
            "shape": "",
            "odom": ""
        }

        self.has_received_img = False

        # -----------------------------
        # ROS2 INIT
        # -----------------------------
        if not rclpy.ok():
            rclpy.init()

        self.camera_node = None
        self.auto_image_mode = False
        self.executor = None
        self.executor_thread = None
        self.auto_image_thread = None

        self._setup_auto_mode()

        if self.camera_node:
            self._start_ros2_threads()

        self.start()

    # =========================================================
    # ROS2 SETUP
    # =========================================================

    def _setup_auto_mode(self):
        try:
            self.camera_node = CameraNode("/webgui_image")
            self.auto_image_mode = True
            print("ROS2 mode enabled: Subscribed to /webgui_image")

        except Exception as e:
            print(f"ROS2 mode disabled: {e}")
            self.camera_node = None
            self.auto_image_mode = False

    def _start_ros2_threads(self):
        try:
            self.executor = rclpy.executors.MultiThreadedExecutor()
            self.executor.add_node(self.camera_node)

            self.executor_thread = threading.Thread(
                target=self.executor.spin,
                daemon=True,
                name="webgui_ros2_executor"
            )
            self.executor_thread.start()

            self.auto_image_thread = threading.Thread(
                target=self._unified_image_loop,
                daemon=True,
                name="webgui_image_display",
            )
            self.auto_image_thread.start()

        except Exception as e:
            print(f"Error starting ROS2 threads: {e}", file=sys.stderr)

    def _unified_image_loop(self):
        while True:
            try:
                if self.camera_node:
                    image = self.camera_node.getImage()
                    if image is not None:
                        self.showImage(image.data)

                threading.Event().wait(0.033)

            except Exception as e:
                print(f"Error in image loop: {e}", file=sys.stderr)
                threading.Event().wait(1.0)

    # =========================================================
    # INPUT FROM FRONTEND
    # =========================================================

    def gui_in_thread(self, ws, message):
        TIME_FRAME_SIZE = 20

        if "ack" in message:
            with self.ack_lock:
                self.ack = True

        elif "start" in message:
            with self.ack_lock:
                self.ack_frontend = True

        if "pick" in message:
            self._handle_input_frame(message, TIME_FRAME_SIZE)

        if "introspection" in message:
            info = message[len("introspection:") :]
            try:
                self.fps, self.lat = info.split("/")
            except ValueError:
                print(f"Invalid introspection format: {info}", file=sys.stderr)

    def _handle_input_frame(self, message, time_frame_size):
        try:
            self.has_received_img = True

            base64_buffer = message[4:-time_frame_size]
            timestamp = message[-time_frame_size:]

            if base64_buffer.startswith("data:image/jpeg;base64,"):
                base64_buffer = base64_buffer[len("data:image/jpeg;base64,") :]

            image_data = base64.b64decode(base64_buffer)
            nparr = np.frombuffer(image_data, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if img is None:
                print("Warning: Failed to decode image", file=sys.stderr)
                return

            with self.frame_rgb_lock:
                self.frame_rgb = img

            HAL.publish_input_image(img)

            ack_message = {"ack_img": "ack", "time": timestamp}
            self.send_to_client(json.dumps(ack_message))

        except Exception as e:
            print(f"Error handling frame: {e}", file=sys.stderr)

    # =========================================================
    # GUI OUTPUT
    # =========================================================

    def update_gui(self):
        payload = self.payloadImage()

        self.payload["image"] = json.dumps(payload["image"])
        self.payload["shape"] = json.dumps(payload["shape"])
        self.payload["odom"] = json.dumps(self.payload.get("odom", ""))

        message = json.dumps(self.payload)
        self.send_to_client(message)

        if not self.has_received_img:
            self.send_to_client(json.dumps({"ack_img": "ack", "time": ""}))

    def payloadImage(self):
        with self.image_show_lock:
            image = self.image_to_be_shown

        payload = {"image": "", "shape": ""}

        if image is None:
            return payload

        try:
            shape = image.shape
            success, frame = cv2.imencode(".JPEG", image)

            if not success:
                return payload

            encoded = base64.b64encode(frame).decode("utf-8")

            payload["image"] = encoded
            payload["shape"] = shape

        except Exception as e:
            print(f"Encoding error: {e}", file=sys.stderr)

        return payload

    # =========================================================
    # PUBLIC API (Python mode)
    # =========================================================

    def showImage(self, image):
        if image is None:
            return

        with self.image_show_lock:
            self.image_to_be_shown = image
            self.image_to_be_shown_updated = True

    def getImage(self):
        with self.frame_rgb_lock:
            return self.frame_rgb

    # =========================================================
    # NEW: ODOMETRY CHANNEL
    # =========================================================

    def setOdom(self, odom):
        """
        Store odometry data to be sent to frontend

        Args:
            odom: dict {x, y, drift}
        """
        try:
            self.payload["odom"] = odom
        except Exception as e:
            print(f"Error setting odom: {e}", file=sys.stderr)


# =========================================================
# GLOBAL INSTANCE
# =========================================================

host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

start_console()

# =========================================================
# EXPOSE API
# =========================================================

def showImage(image):
    if gui:
        gui.showImage(image)


def getImage():
    if gui:
        return gui.getImage()
    return None


def sendOdom(odom):
    """
    Send odometry data to WebGUI

    Args:
        odom: dict {x, y, drift}
    """
    if gui:
        gui.setOdom(odom)