"""
WebGUI for Visual Odometry 3D
Server is PASSIVE: only forwards raw images and odom to client.
No overlay or trajectory drawing on the server.
"""

import json
import cv2
import base64
import threading
import rclpy
import sys
import numpy as np
import traceback
from pathlib import Path

from gui_interfaces.general.measuring_threading_gui_no_sim import MeasuringThreadingGUI
from console_interfaces.general.console import start_console
from hal_interfaces.general.camera import CameraNode

import HAL


# =========================================================
# POSES LOADER (FIXED PATH)
# =========================================================
def load_poses():
    """
    Ruta FIJA como pediste:
    /RoboticsAcademy/exercises/visual_odom/frontend/resources/poses.txt
    """
    file_path = Path("/RoboticsAcademy/exercises/visual_odom/frontend/resources/poses.txt")

    print("[GT] Looking for:", file_path)

    if not file_path.exists():
        print("[GT] ERROR: poses.txt not found at fixed path")
        return []

    poses = []
    with open(file_path, "r") as f:
        for i, line in enumerate(f):
            try:
                vals = list(map(float, line.strip().split()))
                T = np.array(vals).reshape(3, 4)

                pos = [float(T[0, 3]), float(T[1, 3]), float(T[2, 3])]
                poses.append(pos)

                if i < 3:
                    print(f"[GT] sample {i}: {pos}")

            except Exception as e:
                print(f"[GT] parse error line {i}: {e}")

    print(f"[GT] TOTAL POSES LOADED: {len(poses)}")
    return poses


# =========================================================
# WEBGUI CLASS
# =========================================================
class WebGUI(MeasuringThreadingGUI):

    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        print("[WEBGUI] INIT START")

        # -----------------------------
        # IMAGE STATE
        # -----------------------------
        self.image_to_show = None
        self.image_lock = threading.Lock()

        # -----------------------------
        # GROUND TRUTH
        # -----------------------------
        self.gt_positions = load_poses()
        self.gt_path = []
        self.gt_index = 0

        # -----------------------------
        # PAYLOAD
        # -----------------------------
        self.payload = {
            "image": "",
            "shape": "",
            "odom": {"x": 0.0, "y": 0.0, "yaw": 0.0},

            "camera_position": [0.0, 0.0, 0.0],
            "camera_rotation": None,

            "estimated_path": [],
            "ground_truth_path": [],

            "pts": None,
            "pts3d": None,
            "pts_kind": None,
            "ts": None
        }

        # -----------------------------
        # ROS
        # -----------------------------
        if not rclpy.ok():
            rclpy.init()

        self.camera_node = None
        self.executor = None

        self._setup_ros()
        self._start_threads()

        self.start()

        print("[WEBGUI] INIT COMPLETE")

    # =========================================================
    # ROS
    # =========================================================
    def _setup_ros(self):
        try:
            self.camera_node = CameraNode("/webgui_image")
            print("[ROS] subscribed to /webgui_image")
        except Exception as e:
            print("[ROS ERROR]", e)
            self.camera_node = None

    def _start_threads(self):
        if self.camera_node:
            self.executor = rclpy.executors.MultiThreadedExecutor()
            self.executor.add_node(self.camera_node)

            threading.Thread(target=self.executor.spin, daemon=True).start()
            threading.Thread(target=self._image_loop, daemon=True).start()

    def _image_loop(self):
        while True:
            try:
                if self.camera_node:
                    img = self.camera_node.getImage()
                    if img is not None:
                        self.showImage(img.data)
            except Exception as e:
                print("[IMG LOOP ERROR]", e)

            threading.Event().wait(0.033)

    # =========================================================
    # GT STEP (manual or auto)
    # =========================================================
    def step_gt(self):
        if self.gt_index >= len(self.gt_positions):
            print("[GT] END")
            return

        pos = self.gt_positions[self.gt_index]
        self.gt_index += 1

        self.gt_path.append(pos)

        print(f"[GT] STEP {self.gt_index}: {pos}")

    # =========================================================
    # UPDATE GUI (IMPORTANT PART FIXED)
    # =========================================================
    def update_gui(self):

        payload = self._encode_image()

        try:
            self.payload["image"] = json.dumps(payload["image"])
            self.payload["shape"] = json.dumps(payload["shape"])
        except:
            self.payload["image"] = ""
            self.payload["shape"] = ""

        # -----------------------------
        # IMPORTANT: CLEAN SERIALIZATION
        # -----------------------------
        self.payload["ground_truth_path"] = [
            [float(x), float(y), float(z)]
            for x, y, z in self.gt_path
        ]

        self.payload["estimated_path"] = [
            [float(x), float(y), float(z)]
            for x, y, z in self.payload["estimated_path"]
            if isinstance(x, (list, tuple)) and len(x) == 3
        ]

        # DEBUG
        print("[SEND GT]", len(self.payload["ground_truth_path"]))

        try:
            self.send_to_client(json.dumps(self.payload))
        except Exception as e:
            print("[SEND ERROR]", e)
            traceback.print_exc()

    # =========================================================
    # IMAGE
    # =========================================================
    def _encode_image(self):
        with self.image_lock:
            image = self.image_to_show

        payload = {"image": "", "shape": ""}

        if image is None:
            return payload

        try:
            success, frame = cv2.imencode(".jpg", image)
            if success:
                payload["image"] = base64.b64encode(frame).decode("utf-8")
                payload["shape"] = image.shape
        except:
            pass

        return payload

    def showImage(self, image):
        with self.image_lock:
            self.image_to_show = image.copy()


# =========================================================
# GLOBAL
# =========================================================
host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

start_console()


# =========================================================
# API WRAPPERS
# =========================================================
def showImage(image):
    gui.showImage(image)


def stepGroundTruth():
    gui.step_gt()