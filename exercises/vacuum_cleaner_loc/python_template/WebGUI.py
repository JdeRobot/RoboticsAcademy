import json
import cv2
import base64
import numpy as np
import threading
import rclpy
import math
import sys

from PIL import Image
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image as ROSImage

from gui_interfaces.general.measuring_threading_gui_harmonic import (
    MeasuringThreadingGUI,
)
from console_interfaces.general.console import start_console
from map import Map

red = [0, 0, 255]
orange = [0, 165, 255]
yellow = [0, 255, 255]
green = [0, 255, 0]
blue = [255, 0, 0]
indigo = [130, 0, 75]
violet = [211, 0, 148]


class ROS2BridgeNode(Node):
    def __init__(self):
        super().__init__("gui_data")

        self.pose = None
        self.pose_lock = threading.Lock()

        self.user_map = None
        self.user_map_updated = False
        self.user_map_lock = threading.Lock()

        occ_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        user_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.occ_map_publisher = self.create_publisher(
            ROSImage,
            "/webgui_occ_map",
            occ_qos,
        )

        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        self.create_subscription(
            ROSImage,
            "/webgui_user_map",
            self.user_map_callback,
            user_qos,
        )

    def odom_callback(self, msg):
        with self.pose_lock:
            self.pose = msg.pose.pose

    def user_map_callback(self, msg):
        image = self.ros_image_to_numpy(msg)
        if image is None:
            return

        with self.user_map_lock:
            self.user_map = image
            self.user_map_updated = True

    def ros_image_to_numpy(self, msg):
        try:
            data = np.frombuffer(msg.data, dtype=np.uint8)

            if msg.encoding == "mono8":
                expected_size = msg.height * msg.width
                if data.size != expected_size:
                    self.get_logger().error(
                        f"mono8 size mismatch: expected {expected_size}, got {data.size}"
                    )
                    return None
                return data.reshape((msg.height, msg.width)).copy()

            if msg.encoding == "rgb8":
                expected_size = msg.height * msg.width * 3
                if data.size != expected_size:
                    self.get_logger().error(
                        f"rgb8 size mismatch: expected {expected_size}, got {data.size}"
                    )
                    return None
                img = data.reshape((msg.height, msg.width, 3))
                return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            if msg.encoding == "bgr8":
                expected_size = msg.height * msg.width * 3
                if data.size != expected_size:
                    self.get_logger().error(
                        f"bgr8 size mismatch: expected {expected_size}, got {data.size}"
                    )
                    return None
                return data.reshape((msg.height, msg.width, 3)).copy()

            self.get_logger().warn(
                f"Unsupported image encoding received: {msg.encoding}"
            )
            return None

        except Exception as e:
            self.get_logger().error(f"Error converting ROS image: {e}")
            return None

    def publish_occ_map(self, image):
        if image is None:
            return

        msg = ROSImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.height = image.shape[0]
        msg.width = image.shape[1]
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = image.shape[1] * 3
        msg.data = image.astype(np.uint8).tobytes()
        self.occ_map_publisher.publish(msg)

    def get_pose3d(self):
        with self.pose_lock:
            if self.pose is None:

                class Pose3D:
                    x = 0.0
                    y = 0.0
                    yaw = 0.0

                return Pose3D()

            q = self.pose.orientation
            yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z),
            )

            class Pose3D:
                def __init__(self, x, y, yaw):
                    self.x = x
                    self.y = y
                    self.yaw = yaw

            return Pose3D(self.pose.position.x, self.pose.position.y, yaw)

    def pop_user_map(self):
        with self.user_map_lock:
            if self.user_map is None or not self.user_map_updated:
                return None
            image = self.user_map.copy()
            self.user_map_updated = False
            return image


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()

        self.payload = {"map": "", "user": ""}
        self.init_coords = (171, 63)
        self.start_coords = (201, 85.5)

        self.ros_node = None
        self.executor = None
        self.executor_thread = None
        self.image_thread = None

        self._setup_ros2()

        pose_getter = (
            self.ros_node.get_pose3d if self.ros_node is not None else self._dummy_pose
        )
        self.map = Map(pose_getter)

        self._publish_initial_occ_map()

        self.start()

    def _setup_ros2(self):
        try:
            if not rclpy.ok():
                rclpy.init()

            self.ros_node = ROS2BridgeNode()
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.ros_node)

            self.executor_thread = threading.Thread(
                target=self._ros_spin,
                daemon=True,
                name="webgui_ros2_executor",
            )
            self.executor_thread.start()

            self.image_thread = threading.Thread(
                target=self._image_loop,
                daemon=True,
                name="webgui_ros2_image_loop",
            )
            self.image_thread.start()

            print("WebGUI: ROS2 activated")

        except Exception as e:
            print(f"GUI: ROS2 setup failed: {e}", file=sys.stderr)
            self.ros_node = None
            self.executor = None
            self.executor_thread = None
            self.image_thread = None

    def _publish_initial_occ_map(self):
        occ_map = self.getMap(
            "/resources/exercises/vacuum_cleaner_loc/images/mapgrannyannie.png"
        )
        if occ_map is None:
            print("GUI: Failed to load occupancy map", file=sys.stderr)
            return

        if self.ros_node is not None:
            self.ros_node.publish_occ_map(occ_map)
            print("GUI: Occupancy map published to /webgui_occ_map")

    def _dummy_pose(self):
        class Pose3D:
            x = 0.0
            y = 0.0
            yaw = 0.0

        return Pose3D()

    def _ros_spin(self):
        try:
            self.executor.spin()
        except Exception as e:
            print(f"GUI: ROS2 executor error: {e}", file=sys.stderr)

    def _image_loop(self):
        # The GUI owns the base map and only forwards the user-selected view to the frontend.
        while True:
            try:
                if self.ros_node is not None:
                    user_map = self.ros_node.pop_user_map()
                    if user_map is not None:
                        with self.image_show_lock:
                            self.image_to_be_shown = self.process_colors(user_map)
                            self.image_to_be_shown_updated = True

                threading.Event().wait(0.033)

            except Exception as e:
                print(f"GUI: ROS2 image loop error: {e}", file=sys.stderr)
                threading.Event().wait(1.0)

    def update_gui(self):
        pos_message = self.map.getRobotCoordinates()
        if pos_message == self.init_coords:
            pos_message = self.start_coords

        ang_message = self.map.getRobotAngle()
        pos_message = str(pos_message + ang_message)
        self.payload["map"] = pos_message

        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)

        message = json.dumps(self.payload)
        self.send_to_client(message)

    def payloadImage(self):
        with self.image_show_lock:
            image_to_be_shown_updated = self.image_to_be_shown_updated
            image_to_be_shown = self.image_to_be_shown

        image = image_to_be_shown
        payload = {"image": "", "shape": ""}

        if not image_to_be_shown_updated or image is None:
            return payload

        shape = image.shape
        frame = cv2.imencode(".JPEG", image)[1]
        encoded_image = base64.b64encode(frame)

        payload["image"] = encoded_image.decode("utf-8")
        payload["shape"] = shape

        with self.image_show_lock:
            self.image_to_be_shown_updated = False

        return payload

    def process_colors(self, image):
        if len(image.shape) == 3:
            return image

        colored_image = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)

        mask = image < 128
        colored_image[mask] = image[mask][:, None] * 2

        color_table = {
            128: red,
            129: orange,
            130: yellow,
            131: green,
            132: blue,
            133: indigo,
            134: violet,
        }

        for value, color in color_table.items():
            mask = image == value
            colored_image[mask] = color

        return colored_image

    def showNumpy(self, image):
        with self.image_show_lock:
            self.image_to_be_shown = self.process_colors(image)
            self.image_to_be_shown_updated = True

    def getMap(self, url):
        try:
            with Image.open(url) as img:
                img = img.convert("RGB")
                img_array = np.array(img)
            return img_array
        except Exception as e:
            print(f"Error reading image from {url}: {e}")
            return None

    def reset_gui(self):
        self.map.reset()

    def __del__(self):
        try:
            if self.executor:
                self.executor.shutdown()
        except Exception:
            pass


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

start_console()


def showNumpy(image):
    gui.showNumpy(image)


def getMap(url):
    return gui.getMap(url)
