"""
WebGUI for Visual Odometry 3D
Server is PASSIVE: only forwards raw images and odom to client.
No overlay or trajectory drawing on the server.

EXTENSION:
- camera_position (3D)
- camera_rotation
- estimated_path (3D)
- ground_truth_path (3D)
"""

import json
import cv2
import base64
import threading
import rclpy
import sys
import numpy as np
import time
import traceback

from gui_interfaces.general.measuring_threading_gui_no_sim import MeasuringThreadingGUI
from console_interfaces.general.console import start_console
from hal_interfaces.general.camera import CameraNode

import HAL


# =========================================================
# WEBGUI CLASS
# =========================================================
class WebGUI(MeasuringThreadingGUI):

    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        # -----------------------------
        # IMAGE STATE
        # -----------------------------
        self.image_to_show = None
        self.image_lock = threading.Lock()

        self.frame_rgb = None
        self.frame_lock = threading.Lock()

        # -----------------------------
        # INITIAL PAYLOAD (3D SYSTEM)
        # -----------------------------
        self.payload = {
            "image": "",
            "shape": "",

            # 2D LEGACY (NO ROMPER COMPATIBILIDAD)
            "odom": {
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0
            },

            # 3D CAMERA STATE
            "camera_position": [0.0, 0.0, 0.0],
            "camera_rotation": None,

            # TRAJECTORIES
            "estimated_path": [],
            "ground_truth_path": [],

            # optional metadata
            "pts": None,
            "pts3d": None,
            "pts_kind": None,
            "ts": None
        }

        self.has_image = False

        # -----------------------------
        # ROS2 INIT
        # -----------------------------
        if not rclpy.ok():
            rclpy.init()

        self.camera_node = None
        self.executor = None

        self._setup_ros()
        self._start_threads()

        self.start()

    # =========================================================
    # ROS SETUP
    # =========================================================
    def _setup_ros(self):
        try:
            self.camera_node = CameraNode("/webgui_image")
            print("WebGUI subscribed to /webgui_image")
        except Exception as e:
            print(f"ROS disabled: {e}")
            self.camera_node = None

    def _start_threads(self):
        if self.camera_node:
            self.executor = rclpy.executors.MultiThreadedExecutor()
            self.executor.add_node(self.camera_node)

            threading.Thread(
                target=self.executor.spin,
                daemon=True
            ).start()

            threading.Thread(
                target=self._image_loop,
                daemon=True
            ).start()

    def _image_loop(self):
        while True:
            try:
                if self.camera_node:
                    img = self.camera_node.getImage()
                    if img is not None:
                        self.showImage(img.data)
            except Exception as e:
                print(f"Image loop error: {e}", file=sys.stderr)

            threading.Event().wait(0.033)

    # =========================================================
    # INPUT FROM FRONTEND
    # =========================================================
    def gui_in_thread(self, ws, message):
        TIME_FRAME_SIZE = 20

        if "pick" in message:
            self._handle_frame(message, TIME_FRAME_SIZE)

    def _handle_frame(self, message, time_frame_size):
        try:
            self.has_image = True

            base64_buffer = message[4:-time_frame_size]
            timestamp = message[-time_frame_size:]

            if base64_buffer.startswith("data:image/jpeg;base64,"):
                base64_buffer = base64_buffer[len("data:image/jpeg;base64,"):]

            image_data = base64.b64decode(base64_buffer)
            nparr = np.frombuffer(image_data, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if img is None:
                return

            with self.frame_lock:
                self.frame_rgb = img

            try:
                HAL.publish_input_image(img)
            except Exception:
                traceback.print_exc()

            self.send_to_client(json.dumps({
                "ack_img": "ok",
                "time": timestamp
            }))

        except Exception as e:
            print(f"Frame error: {e}", file=sys.stderr)
            traceback.print_exc()

    # =========================================================
    # OUTPUT LOOP
    # =========================================================
    def update_gui(self):
        payload = self._encode_image()

        try:
            self.payload["image"] = json.dumps(payload["image"])
            self.payload["shape"] = json.dumps(payload["shape"])
        except Exception:
            self.payload["image"] = json.dumps("")
            self.payload["shape"] = json.dumps("")

        # -----------------------------
        # PROTECT 3D DATA (NO OVERWRITE)
        # -----------------------------
        protected_keys = {
            "camera_position",
            "camera_rotation",
            "estimated_path",
            "ground_truth_path",
            "pts",
            "pts3d",
            "pts_kind",
            "ts"
        }

        # serialize safe fields only
        for k in list(self.payload.keys()):
            if k in protected_keys:
                continue

            try:
                val = self.payload[k]

                if isinstance(val, str):
                    try:
                        json.loads(val)
                        message_val = val
                    except Exception:
                        message_val = json.dumps(val)
                else:
                    message_val = json.dumps(val)

                self.payload[k] = message_val

            except Exception:
                traceback.print_exc()

        try:
            message = json.dumps(self.payload)
            self.send_to_client(message)
        except Exception as e:
            print(f"Failed to send payload: {e}", file=sys.stderr)
            traceback.print_exc()

    # =========================================================
    # IMAGE ENCODING
    # =========================================================
    def _encode_image(self):
        with self.image_lock:
            image = self.image_to_show

        payload = {"image": "", "shape": ""}

        if image is None:
            return payload

        try:
            success, frame = cv2.imencode(".jpg", image)
            if not success:
                return payload

            encoded = base64.b64encode(frame).decode("utf-8")

            payload["image"] = encoded
            payload["shape"] = image.shape

        except Exception as e:
            print(f"Encoding error: {e}", file=sys.stderr)
            traceback.print_exc()

        return payload

    # =========================================================
    # PUBLIC API
    # =========================================================
    def showImage(self, image):
        if image is None:
            return

        with self.image_lock:
            try:
                self.image_to_show = image.copy()
            except Exception:
                self.image_to_show = image

    def getImage(self):
        with self.frame_lock:
            return self.frame_rgb


# =========================================================
# GLOBAL INSTANCE
# =========================================================
host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

start_console()


# =========================================================
# API WRAPPERS
# =========================================================
def showImage(image):
    if gui:
        gui.showImage(image)

def getImage():
    if gui:
        return gui.getImage()
    return None

def sendOdom(odom):
    try:
        if gui:
            gui.payload["odom"] = {
                "x": float(odom.get("x", 0.0)),
                "y": float(odom.get("y", 0.0)),
                "yaw": float(odom.get("yaw", 0.0))
            }
    except Exception:
        traceback.print_exc()


# =========================================================
# NEW 3D API
# =========================================================
def publishPose3D(position, rotation=None):
    if gui:
        gui.payload["camera_position"] = [
            float(position[0]),
            float(position[1]),
            float(position[2])
        ]
        if rotation is not None:
            gui.payload["camera_rotation"] = rotation


def publishEstimatedPath(path):
    if gui:
        gui.payload["estimated_path"] = [
            [float(x), float(y), float(z)] for x, y, z in path
        ]


def publishGroundTruthPath(path):
    if gui:
        gui.payload["ground_truth_path"] = [
            [float(x), float(y), float(z)] for x, y, z in path
        ]


def publishAll(position, estimated_path, gt_path):
    if gui:
        gui.payload["camera_position"] = [
            float(position[0]),
            float(position[1]),
            float(position[2])
        ]
        gui.payload["estimated_path"] = estimated_path
        gui.payload["ground_truth_path"] = gt_path