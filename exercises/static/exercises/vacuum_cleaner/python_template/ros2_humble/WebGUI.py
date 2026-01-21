import json
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry
from gui_interfaces.general.measuring_threading_gui_harmonic import (
    MeasuringThreadingGUI,
)
from console_interfaces.general.console import start_console
from map import Map

# Simple HAL check
try:
    from HAL import getPose3d as hal_getPose3d
    HAL_AVAILABLE = True
except ImportError:
    HAL_AVAILABLE = False


class ROS2Node(Node):
    def __init__(self):
        super().__init__("gui_data")
        self.pose = None
        self.new_pose = False
        self.pose_lock = threading.Lock()
        self.create_subscription(Odometry, "/odom", self.callback, 10)

    def callback(self, msg):
        with self.pose_lock:
            self.pose = msg.pose.pose
            self.new_pose = True

    def consume_pose(self):
        with self.pose_lock:
            if not self.new_pose:
                return None
            self.new_pose = False
            return self.pose

    def get_pose3d(self):
        with self.pose_lock:
            if not self.pose:
                class P:
                    x = y = yaw = 0.0
                return P()

            import math
            q = self.pose.orientation
            yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z),
            )

            class Pose3D:
                def __init__(self, x, y, yaw):
                    self.x, self.y, self.yaw = x, y, yaw

            return Pose3D(self.pose.position.x, self.pose.position.y, yaw)


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        # Payload vars (KEEP ORIGINAL PROTOCOL)
        self.payload = {"map": ""}
        self.init_coords = (171, 63)
        self.start_coords = (201, 85.5)

        # ROS2 components
        self.ros_node = None
        self.executor = None
        self.executor_thread = None

        # Auto-detect mode and get pose function
        pose_getter = self._get_pose_function()
        self.map = Map(pose_getter)

        # Start GUI (IMPORTANT)
        self.start()

    def _get_pose_function(self):
        """Auto-detect the best pose source"""
        try:
            if not rclpy.ok():
                rclpy.init()

            temp_node = rclpy.create_node("temp_topic_checker")
            topics = [name for name, _ in temp_node.get_topic_names_and_types()]
            temp_node.destroy_node()

            if "/odom" in topics:
                self.ros_node = ROS2Node()
                self.executor = SingleThreadedExecutor()
                self.executor.add_node(self.ros_node)

                self.executor_thread = threading.Thread(
                    target=self._ros_spin, daemon=True
                )
                self.executor_thread.start()

                print("GUI: ROS2 mode activated - listening to /odom")
                return self.ros_node.get_pose3d

        except Exception as e:
            print(f"GUI: ROS2 setup failed: {e}")

        if HAL_AVAILABLE:
            print("GUI: HAL mode activated")
            return hal_getPose3d

        print("GUI: Dummy mode activated")

        class DummyPose:
            x = y = yaw = 0.0

        return lambda: DummyPose()

    def _ros_spin(self):
        try:
            self.executor.spin()
        except Exception as e:
            print(f"GUI: ROS2 executor error: {e}")

    # ✅ FIXED: GUI updates synchronized to /odom
    def update_gui(self):
        # Only update GUI when a NEW odom message arrives
        if self.ros_node:
            pose = self.ros_node.consume_pose()
            if pose is None:
                return

        pos = self.map.getRobotCoordinates()
        if pos == self.init_coords:
            pos = self.start_coords

        ang = self.map.getRobotAngle()

        # KEEP ORIGINAL MAP STRING FORMAT
        self.payload["map"] = str(pos + ang)
        self.send_to_client(json.dumps(self.payload))

    def reset_gui(self):
        self.map.reset()

    def __del__(self):
        if self.executor:
            self.executor.shutdown()


# Create GUI instance
gui = WebGUI()
start_console()
