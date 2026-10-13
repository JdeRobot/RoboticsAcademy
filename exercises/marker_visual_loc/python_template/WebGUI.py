import json
import subprocess
import cv2
import base64
import threading
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge

from map import Map
from gui_interfaces.general.measuring_threading_gui_harmonic import (
    MeasuringThreadingGUI,
)
from console_interfaces.general.console import start_console


def quat_to_yaw(qw, qx, qy, qz):
    rotate_za0 = 2.0 * (qx * qy + qw * qz)
    rotate_za1 = qw * qw + qx * qx - qy * qy - qz * qz
    if rotate_za0 != 0.0 and rotate_za1 != 0.0:
        return math.atan2(rotate_za0, rotate_za1)
    return 0.0


class Pose3d:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, timeStamp=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.timeStamp = timeStamp


class ROS2BridgeNode(Node):
    def __init__(self, gui_instance):
        super().__init__("gui_bridge_node_filter")
        self.gui = gui_instance
        self.bridge = CvBridge()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(Odometry, "/turtlebot3/odom", self.pose3d_callback, 10)
        self.create_subscription(
            Odometry, "/turtlebot3/odom_noisy", self.odom_callback, 10
        )
        self.create_subscription(
            PoseStamped, "/webgui/estimated_pose", self.estimated_pose_callback, qos
        )
        self.create_subscription(
            RosImage, "/webgui/image_debug", self.image_callback, 10
        )

        self.pose3d = Pose3d()
        self.odom = Pose3d()

    def pose3d_callback(self, msg):
        self.pose3d.x = msg.pose.pose.position.x
        self.pose3d.y = msg.pose.pose.position.y
        ori = msg.pose.pose.orientation
        self.pose3d.yaw = quat_to_yaw(ori.w, ori.x, ori.y, ori.z)
        self.pose3d.timeStamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def odom_callback(self, msg):
        self.odom.x = msg.pose.pose.position.x
        self.odom.y = msg.pose.pose.position.y
        ori = msg.pose.pose.orientation
        self.odom.yaw = quat_to_yaw(ori.w, ori.x, ori.y, ori.z)
        self.odom.timeStamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def estimated_pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        ori = msg.pose.orientation
        yaw = quat_to_yaw(ori.w, ori.x, ori.y, ori.z)
        self.gui.showEstimatedPose((x, y, yaw))

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.gui.setImage(cv_image)


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303", freq=30.0):
        super().__init__(host)
        self.image = None
        self.image_lock = threading.Lock()
        self.predict_pose = None

        self.bridge_node = None
        self.executor = None
        self.executor_thread = None

        self._setup_ros2()
        self.map = Map(self.get_pose3d, self.get_odom)

        self.payload = {
            "image": "",
            "real_pose": "",
            "noisy_pose": "",
            "estimate_pose": "",
        }

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

    def get_pose3d(self):
        return self.bridge_node.pose3d

    def get_odom(self):
        return self.bridge_node.odom

    def update_gui(self):
        if self.map.getRobotCoordinates() is not None:
            pos_message = self.map.getRobotCoordinates()
            pos_message = str(pos_message)
            self.payload["real_pose"] = pos_message

        if self.map.getRobotCoordinatesWithNoise() is not None:
            n_pos_message = self.map.getRobotCoordinatesWithNoise()
            n_pos_message = str(n_pos_message)
            self.payload["noisy_pose"] = n_pos_message

        if np.any(self.predict_pose):
            p_pos_message = str(self.predict_pose)
            self.payload["estimate_pose"] = p_pos_message

        if np.any(self.image):
            _, encoded_image = cv2.imencode(".JPEG", self.image)
            b64 = base64.b64encode(encoded_image).decode("utf-8")
            shape = self.image.shape
        else:
            b64 = None
            shape = 0

        payload_img = {
            "image": b64,
            "shape": shape,
        }

        self.payload["image"] = json.dumps(payload_img)
        message = json.dumps(self.payload)
        self.send_to_client(message)

    def setImage(self, image):
        with self.image_lock:
            self.image = image

    def setEstimatedRobotPose(self, pose):
        self.predict_pose = pose

    def showEstimatedPose(self, pose):
        x, y, yaw = pose
        scale_y = -85
        offset_y = -6.88
        y = scale_y * (offset_y - y)
        scale_x = 83
        offset_x = 8
        x = scale_x * (offset_x - x)
        transformed_pose = (x, y, yaw)
        self.setEstimatedRobotPose(transformed_pose)

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
    gui.setImage(image)


def showEstimatedPose(pose):
    gui.showEstimatedPose(pose)
