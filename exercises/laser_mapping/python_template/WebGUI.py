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

from map import Map
from gui_interfaces.general.measuring_threading_gui_harmonic import MeasuringThreadingGUI
from console_interfaces.general.console import start_console

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
            Odometry, "/turtlebot3/odom", self.odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "/turtlebot3/odom_noisy", self.noisy_odom_callback, qos_profile_sensor_data
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
        self.payload = {"user_map": "", "real_pose": "", "noisy_pose": ""}

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
        return Pose3d(self.bridge_node.pose.x, self.bridge_node.pose.y, self.bridge_node.pose.yaw)

    def get_noisy_pose(self):
        return Pose3d(self.bridge_node.noisy_pose.x, self.bridge_node.noisy_pose.y, self.bridge_node.noisy_pose.yaw)

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
        message = json.dumps(self.payload)
        self.send_to_client(message)

    def setUserMap(self, image):
        if image.shape[0] != 970 or image.shape[1] != 1500:
            raise ValueError(
                "map passed has the wrong dimensions, it has to be 970 pixels high and 1500 pixels wide"
            )

        if len(image.shape) == 2:
            processed_image = np.stack((image,) * 3, axis=-1)
        else:
            processed_image = image

        with self.image_lock:
            self.user_map = processed_image

    def poseToMap(self, x_prime, y_prime, yaw_prime):
        y = -23.58 * (-20.36 - x_prime)
        x = -23.53 * (-31.95 - y_prime)
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