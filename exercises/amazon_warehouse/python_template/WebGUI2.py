import json
import threading
import sys
import math
import cv2
import re
import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image as ROSImage
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String

from gui_interfaces.general.measuring_threading_gui import MeasuringThreadingGUI
from map import Map
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
        self.lift_state = False

        qos_transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(Odometry, "/amazon_robot/odom", self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(Float64, "/platform/cmd_vel", self.lift_callback, 10)
        self.create_subscription(String, "/webgui/path", self.path_callback, qos_transient)

        self.map1_pub = self.create_publisher(ROSImage, "/webgui/map_1", qos_transient)
        self.map2_pub = self.create_publisher(ROSImage, "/webgui/map_2", qos_transient)
        
        self.publish_maps()

    def publish_maps(self):
        try:
            map1 = cv2.imread("/resources/exercises/amazon_warehouse/images/map.png")
            map2 = cv2.imread("/resources/exercises/amazon_warehouse/images/map_2.png")
            
            if map1 is not None:
                msg1 = ROSImage()
                msg1.height, msg1.width, _ = map1.shape
                msg1.encoding = "bgr8"
                msg1.step = msg1.width * 3
                msg1.data = np.array(map1, dtype=np.uint8).tobytes()
                self.map1_pub.publish(msg1)
                
            if map2 is not None:
                msg2 = ROSImage()
                msg2.height, msg2.width, _ = map2.shape
                msg2.encoding = "bgr8"
                msg2.step = msg2.width * 3
                msg2.data = np.array(map2, dtype=np.uint8).tobytes()
                self.map2_pub.publish(msg2)
        except Exception:
            pass

    def odom_callback(self, msg):
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        ori = msg.pose.pose.orientation
        self.pose.yaw = quat_to_yaw(ori.w, ori.x, ori.y, ori.z)

    def lift_callback(self, msg):
        self.lift_state = msg.data > 0.0

    def path_callback(self, msg):
        self.gui.update_path_array(msg.data)


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        self.array_lock = threading.Lock()
        self.array = ""
        self.payload = {"map": "", "array": "", "liftState": False}

        self.bridge_node = None
        self.executor = None
        self.executor_thread = None

        self._setup_ros2()

        self.map = Map(self.get_pose3d)

        self.init_coords = self.map.getRobotCoordinates()
        self.start_coords = self.map.getRobotCoordinates()

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
        return self.bridge_node.pose

    def get_lift_state(self):
        return self.bridge_node.lift_state

    def update_gui(self):
        with self.array_lock:
            self.payload["array"] = self.array
        
        self.payload["liftState"] = self.get_lift_state()

        pos_message = self.map.getRobotCoordinates()
        ang_message = self.map.getRobotAngle()
        self.payload["map"] = str(pos_message + ang_message)

        message = json.dumps(self.payload)
        self.send_to_client(message)

    def update_path_array(self, strArray):
        with self.array_lock:
            self.array = strArray

    def showPath(self, array):
        array_scaled = []
        for wp in array:
            array_scaled.append([wp[0] * 0.72, wp[1] * 0.545])

        strArray = "".join(str(e) for e in array_scaled)
        strArray = re.sub(r"\[[ ]+", "[", strArray)
        strArray = re.sub(r"[ ]+", ", ", strArray)
        strArray = re.sub(r",[ ]+]", "]", strArray)
        strArray = re.sub(r",,", ",", strArray)
        strArray = re.sub(r"]\[", "],[", strArray)
        strArray = "[" + strArray + "]"

        with self.array_lock:
            self.array = strArray

    def getMap(self, url):
        return plt.imread(url)

    def __del__(self):
        try:
            if self.executor:
                self.executor.shutdown()
        except Exception:
            pass


if not rclpy.ok():
    rclpy.init(args=sys.argv)

host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

start_console()


def getMap(url):
    return gui.getMap(url)


def showPath(array):
    return gui.showPath(array)