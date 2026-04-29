"""
WebGUI for Basic Computer Vision Exercise
Supports dual mode: Python API (getImage/showImage) and ROS2 topics
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

# Import HAL to trigger ROS2 setup
import exercises.visual_odom.python_template.HAL as HAL


class WebGUI(MeasuringThreadingGUI):
    """
    WebGUI class that bridges browser frontend with Python/ROS2 backends

    Supports two modes:
    1. Python mode: getImage() and showImage() functions
    2. ROS2 mode: /input/image_raw (input) and /webgui_image (output) topics
    """

    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        # Image storage for Python mode
        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()

        # Input storage for Python mode
        self.frame_rgb = None
        self.frame_rgb_lock = threading.Lock()

        # WebSocket payload
        self.payload = {"image": "", "shape": ""}
        self.has_received_img = False

        # =========================
        # POSE (ODOMETRÍA VISUAL)
        # =========================
        self.pose = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "yaw": 0.0,
            "pitch": 0.0,
            "roll": 0.0
        }
        self.pose_lock = threading.Lock()

        # =========================
        # NUEVO: TRAJECTORY (DERIVA)
        # =========================
        self.trajectory = []
        self.traj_lock = threading.Lock()

        # Initialize ROS2 if not already initialized
        if not rclpy.ok():
            rclpy.init()

        # Setup ROS2 subscriber for /webgui_image topic
        self.camera_node = None
        self.auto_image_mode = False
        self.executor = None
        self.executor_thread = None
        self.auto_image_thread = None

        self._setup_auto_mode()

        # Start ROS2 executor and image display thread if subscriber created
        if self.camera_node:
            self._start_ros2_threads()

        self.start()

    # =========================
    # POSE UPDATE (DESDE HAL)
    # =========================
    def updatePose(self, x, y, z, yaw, pitch=0.0, roll=0.0):
        with self.pose_lock:
            self.pose = {
                "x": float(x),
                "y": float(y),
                "z": float(z),
                "yaw": float(yaw),
                "pitch": float(pitch),
                "roll": float(roll)
            }

        # =========================
        # TRAJECTORY UPDATE
        # =========================
        with self.traj_lock:
            self.trajectory.append({
                "x": float(x),
                "y": float(y),
                "z": float(z),
                "yaw": float(yaw)
            })

        print(f"[WebGUI] Pose updated: {self.pose}")

    def _setup_auto_mode(self):
        """Set up automatic image subscription for /webgui_image topic"""
        try:
            self.camera_node = CameraNode("/webgui_image")
            self.auto_image_mode = True
            print("ROS2 mode enabled: Subscribed to /webgui_image")

        except Exception as e:
            print(f"ROS2 mode disabled: Could not subscribe to /webgui_image: {e}")
            self.camera_node = None
            self.auto_image_mode = False

    def _start_ros2_threads(self):
        """Start ROS2 executor and image display threads"""
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
                print(f"Error in image display loop: {e}", file=sys.stderr)
                threading.Event().wait(1.0)

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
                print("Warning: Failed to decode input image", file=sys.stderr)
                return

            with self.frame_rgb_lock:
                self.frame_rgb = img

            HAL.publish_input_image(img)

            ack_message = {"ack_img": "ack", "time": timestamp}
            self.send_to_client(json.dumps(ack_message))

        except Exception as e:
            print(f"Error handling input frame: {e}", file=sys.stderr)

    def update_gui(self):
        payload = self.payloadImage()

        # =========================
        # ADD POSE + TRAJECTORY
        # =========================
        with self.pose_lock:
            payload["pose"] = self.pose.copy()

        with self.traj_lock:
            payload["trajectory"] = self.trajectory.copy()

        self.payload["image"] = json.dumps(payload)

        message = json.dumps(self.payload)
        self.send_to_client(message)

        if not self.has_received_img:
            ack_message = {"ack_img": "ack", "time": ""}
            self.send_to_client(json.dumps(ack_message))

    def payloadImage(self):
        with self.image_show_lock:
            image_to_be_shown = self.image_to_be_shown
            image_to_be_shown_updated = self.image_to_be_shown_updated

        payload = {"image": "", "shape": ""}

        if image_to_be_shown is None:
            return payload

        try:
            shape = image_to_be_shown.shape
            success, frame = cv2.imencode(".JPEG", image_to_be_shown)

            if not success:
                print("Warning: Failed to encode image as JPEG", file=sys.stderr)
                return payload

            encoded_image = base64.b64encode(frame)
            payload["image"] = encoded_image.decode("utf-8")
            payload["shape"] = shape

            if image_to_be_shown_updated:
                with self.image_show_lock:
                    self.image_to_be_shown_updated = False

        except Exception as e:
            print(f"Error encoding image payload: {e}", file=sys.stderr)

        return payload

    def showImage(self, image):
        if image is None:
            print("Warning: Attempted to show None image", file=sys.stderr)
            return

        with self.image_show_lock:
            self.image_to_be_shown = image
            self.image_to_be_shown_updated = True

    def getImage(self):
        with self.frame_rgb_lock:
            return self.frame_rgb


# Create GUI instance
host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

start_console()


def showImage(image):
    if gui is not None:
        gui.showImage(image)
    else:
        print("Error: GUI not initialized", file=sys.stderr)


def getImage():
    if gui is not None:
        return gui.getImage()
    else:
        print("Error: GUI not initialized", file=sys.stderr)
        return None