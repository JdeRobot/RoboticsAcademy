import json
import cv2
import base64
import threading
import numpy as np
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image as ROSImage
from nav_msgs.msg import Odometry

from map import (
    MAP_HEIGHT,
    MAP_WIDTH,
    ORIGIN_X,
    ORIGIN_Y,
    SCALE_X,
    SCALE_Y,
    Map,
    world_to_pixel,
)
from gui_interfaces.general.measuring_threading_gui_harmonic import (
    MeasuringThreadingGUI,
)
from console_interfaces.general.console import start_console

# Nav2 map_server thresholds written in the exported YAML
OCCUPIED_THRESH = 0.65
FREE_THRESH = 0.25
# Unknown pixels kept around the mapped area of the exported map
CROP_MARGIN = 10


def quat_to_yaw(qw, qx, qy, qz):
    rotate_za0 = 2.0 * (qx * qy + qw * qz)
    rotate_za1 = qw * qw + qx * qx - qy * qy - qz * qz
    if rotate_za0 != 0.0 and rotate_za1 != 0.0:
        return math.atan2(rotate_za0, rotate_za1)
    return 0.0


class Pose3d:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw


class GUIBridgeNode(Node):
    def __init__(self, gui_instance):
        super().__init__("gui_bridge_node")
        self.gui = gui_instance
        self.pose = Pose3d()
        self.noisy_pose = Pose3d()

        qos_transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(
            ROSImage, "/webgui/user_map", self.user_map_callback, qos_transient
        )
        self.create_subscription(
            Odometry, "/do150/odom", self.odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry,
            "/do150/odom_noisy",
            self.noisy_odom_callback,
            qos_profile_sensor_data,
        )

    def user_map_callback(self, msg):
        try:
            data = np.frombuffer(msg.data, dtype=np.uint8)
            image = data.reshape((msg.height, msg.width))
            self.gui.setUserMap(image)
        except Exception:
            pass

    def odom_callback(self, msg):
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        ori = msg.pose.pose.orientation
        self.pose.yaw = quat_to_yaw(ori.w, ori.x, ori.y, ori.z)

    def noisy_odom_callback(self, msg):
        self.noisy_pose.x = msg.pose.pose.position.x
        self.noisy_pose.y = msg.pose.pose.position.y
        ori = msg.pose.pose.orientation
        self.noisy_pose.yaw = quat_to_yaw(ori.w, ori.x, ori.y, ori.z)


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303", freq=30.0):
        super().__init__(host)

        self.user_map = None
        self.image_lock = threading.Lock()
        self.payload = {
            "user_map": "",
            "real_pose": "",
            "noisy_pose": "",
            "map_file": "",
        }
        self.map_requested = False

        self.bridge_node = None
        self.executor = None
        self.executor_thread = None

        self._setup_ros2()

        self.map = Map(self.get_pose3d, self.get_noisy_pose)

        self.start()

    def _setup_ros2(self):
        if not rclpy.ok():
            rclpy.init(args=sys.argv)

        self.bridge_node = GUIBridgeNode(self)

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.bridge_node)

        self.executor_thread = threading.Thread(
            target=self.executor.spin, daemon=True, name="webgui_ros2_executor"
        )
        self.executor_thread.start()

    def get_pose3d(self):
        return Pose3d(
            self.bridge_node.pose.x, self.bridge_node.pose.y, self.bridge_node.pose.yaw
        )

    def get_noisy_pose(self):
        return Pose3d(
            self.bridge_node.noisy_pose.x,
            self.bridge_node.noisy_pose.y,
            self.bridge_node.noisy_pose.yaw,
        )

    def update_gui(self):
        pos_message = self.map.getRobotCoordinates()
        self.payload["real_pose"] = str(pos_message)

        n_pos_message = self.map.getRobotCoordinatesWithNoise()
        self.payload["noisy_pose"] = str(n_pos_message)

        if np.any(self.user_map):
            _, encoded_image = cv2.imencode(".JPEG", self.user_map)
            b64 = base64.b64encode(encoded_image).decode("utf-8")
            shape = self.user_map.shape
        else:
            b64 = None
            shape = 0

        payload_img = {
            "user_map": b64,
            "shape": shape,
        }

        self.payload["user_map"] = json.dumps(payload_img)

        # The map file goes out once per download request from the frontend
        if self.map_requested:
            self.map_requested = False
            self.payload["map_file"] = json.dumps(self.export_map())
        else:
            self.payload["map_file"] = ""

        message = json.dumps(self.payload)
        self.send_to_client(message)

    def gui_in_thread(self, ws, message):
        if "download_map" in message:
            self.map_requested = True
        else:
            super().gui_in_thread(ws, message)

    # Builds the map in the Nav2 map_server format cropped to the area the user mapped
    def export_map(self):
        with self.image_lock:
            if self.user_map is None:
                return None
            gray = cv2.cvtColor(self.user_map, cv2.COLOR_BGR2GRAY)

        # Nav2 puts world x along the columns and world y upwards along the rows
        nav2_map = np.flipud(gray.T)

        # Pixels map_server reads as free or occupied, the rest is unknown
        occupancy = 1.0 - nav2_map / 255.0
        known = (occupancy > OCCUPIED_THRESH) | (occupancy < FREE_THRESH)
        if not known.any():
            return None

        rows = np.flatnonzero(known.any(axis=1))
        cols = np.flatnonzero(known.any(axis=0))
        top = max(rows[0] - CROP_MARGIN, 0)
        bottom = min(rows[-1] + CROP_MARGIN, nav2_map.shape[0] - 1)
        left = max(cols[0] - CROP_MARGIN, 0)
        right = min(cols[-1] + CROP_MARGIN, nav2_map.shape[1] - 1)
        cropped = np.ascontiguousarray(nav2_map[top : bottom + 1, left : right + 1])
        _, pgm = cv2.imencode(".pgm", cropped)

        # The origin is the world pose of the lower left corner of the crop
        # and poseToMap rounds so every pixel is centred on its world point
        origin_x = ORIGIN_X + (left - 0.5) / SCALE_X
        origin_y = ORIGIN_Y + (MAP_WIDTH - 1 - bottom - 0.5) / SCALE_Y
        resolution = 2.0 / (SCALE_X + SCALE_Y)
        yaml = (
            "image: map.pgm\n"
            "mode: trinary\n"
            f"resolution: {resolution:.6f}\n"
            f"origin: [{origin_x:.4f}, {origin_y:.4f}, 0.0]\n"
            "negate: 0\n"
            f"occupied_thresh: {OCCUPIED_THRESH}\n"
            f"free_thresh: {FREE_THRESH}\n"
        )
        return {"pgm": base64.b64encode(pgm).decode("utf-8"), "yaml": yaml}

    def setUserMap(self, image):
        if image.shape[0] != MAP_HEIGHT or image.shape[1] != MAP_WIDTH:
            raise ValueError(
                f"map passed has the wrong dimensions, it has to be {MAP_HEIGHT} pixels high and {MAP_WIDTH} pixels wide"
            )

        if len(image.shape) == 2:
            processed_image = np.stack((image,) * 3, axis=-1)
        else:
            processed_image = image

        with self.image_lock:
            self.user_map = processed_image

    def poseToMap(self, x_prime, y_prime, yaw_prime):
        y, x = world_to_pixel(x_prime, y_prime)
        yaw = yaw_prime - math.pi / 2
        return [round(x), round(y), yaw]

    def __del__(self):
        try:
            if self.executor:
                self.executor.shutdown()
        except Exception:
            pass


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

start_console()


def setUserMap(image):
    gui.setUserMap(image)


def poseToMap(x_prime, y_prime, yaw_prime):
    return gui.poseToMap(x_prime, y_prime, yaw_prime)
