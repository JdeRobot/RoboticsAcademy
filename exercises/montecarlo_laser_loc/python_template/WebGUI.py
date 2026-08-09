import json
import math
import matplotlib.pyplot as plt
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseArray

from gui_interfaces.general.measuring_threading_gui_harmonic import (
    MeasuringThreadingGUI,
)
from console_interfaces.general.console import start_console
from map import Map


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


class ROS2BridgeNode(Node):
    def __init__(self, gui_instance):
        super().__init__("gui_bridge_node")
        self.gui = gui_instance
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(
            Odometry, "/vacuum_cleaner/odom", self.odom_callback, 10
        )
        self.create_subscription(
            PoseStamped, "/webgui/estimated_pose", self.estimated_pose_callback, qos
        )
        self.create_subscription(
            PoseArray, "/webgui/particles", self.particles_callback, qos
        )

        self.pose = Pose3d()

    def odom_callback(self, msg):
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        ori = msg.pose.pose.orientation
        self.pose.yaw = quat_to_yaw(ori.w, ori.x, ori.y, ori.z)

    def estimated_pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        ori = msg.pose.orientation
        yaw = quat_to_yaw(ori.w, ori.x, ori.y, ori.z)
        self.gui.showPosition(x, y, yaw)

    def particles_callback(self, msg):
        particles = []
        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y
            yaw = quat_to_yaw(
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
            )
            particles.append([x, y, yaw])
        self.gui.showParticles(particles)


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        self.payload = {"map": "", "user": "", "particles": ""}
        self.init_coords = (171, 63)
        self.start_coords = (201, 85.5)

        self.particles = []
        self.user_position = (0, 0)
        self.user_angle = (0, 0)

        self.bridge_node = None
        self.noise_node = None
        self.executor = None
        self.executor_thread = None

        self._setup_ros2()

        self.map = Map(self.get_pose3d)

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
        return self.bridge_node.pose

    def update_gui(self):
        pos_message = self.map.getRobotCoordinates()
        if pos_message == self.init_coords:
            pos_message = self.start_coords
        ang_message = self.map.getRobotAngle()
        pos_message = str(pos_message + ang_message)
        self.payload["map"] = pos_message

        pos_message_user = self.user_position
        ang_message_user = self.user_angle
        pos_message_user = pos_message_user + ang_message_user
        pos_message_user = str(pos_message_user)
        self.payload["user"] = pos_message_user

        if self.particles:
            self.payload["particles"] = json.dumps(self.particles)
        else:
            self.payload["particles"] = json.dumps([])

        message = json.dumps(self.payload)
        self.send_to_client(message)

    def showPosition(self, x, y, angle):
        scale_y = 15
        offset_y = 63
        y = scale_y * y + offset_y

        scale_x = -30
        offset_x = 171
        x = scale_x * x + offset_x

        self.user_position = x, y
        self.user_angle = (angle,)

    def showParticles(self, particles):
        if particles:
            self.particles = particles
            scale_y = 15
            offset_y = 63
            scale_x = -30
            offset_x = 171
            for particle in self.particles:
                particle[1] = scale_y * particle[1] + offset_y
                particle[0] = scale_x * particle[0] + offset_x
        else:
            self.particles = []

    def getMap(self, url):
        return plt.imread(url)

    def poseToMap(self, x_prime, y_prime, yaw_prime):
        x = 101.1 * (4.2 + y_prime)
        y = 101.1 * (5.7 - x_prime)
        yaw = yaw_prime - math.pi / 2
        return [round(x), round(y), yaw]

    def mapToPose(self, map_x, map_y, map_yaw):
        x = (map_y - 576.27) / -101.1
        y = (map_x - 424.62) / 101.1
        yaw = map_yaw + math.pi / 2
        return [x, y, yaw]

    def __del__(self):
        try:
            if self.executor:
                self.executor.shutdown()
        except Exception:
            pass


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

start_console()


def showPosition(x, y, angle):
    gui.showPosition(x, y, angle)


def showParticles(particles):
    gui.showParticles(particles)


def getMap(url):
    return gui.getMap(url)


def poseToMap(x_prime, y_prime, yaw_prime):
    return gui.poseToMap(x_prime, y_prime, yaw_prime)


def mapToPose(x, y, yaw):
    return gui.mapToPose(x, y, yaw)
