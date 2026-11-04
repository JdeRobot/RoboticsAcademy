import json
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float32, String
from trajectory_msgs.msg import JointTrajectoryPoint

from hal_interfaces.general.camera import CameraNode
from hal_interfaces.general.depth_camera import DepthCameraNode
from hal_interfaces.general.laser import LaserNode
from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from ros2srrc_data.msg import Robpose

sys.path.append(os.path.join(get_package_share_directory("ros2srrc_execution"), "python", "robot"))
from robot import RBT  # noqa: E402

freq = 30.0

MAX_V = 0.5
MAX_W = 0.8
ARM_JOINTS = [
    "ur10_shoulder_pan_joint",
    "ur10_shoulder_lift_joint",
    "ur10_elbow_joint",
    "ur10_wrist_1_joint",
    "ur10_wrist_2_joint",
    "ur10_wrist_3_joint",
]
# Folded over the cabinet, the pose the robot starts in
ARM_HOME = [1.5708, -2.75, 2.67, -1.5708, -1.5708, 0.0]
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 0.20


def _transform(xyz, rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    t = np.eye(4)
    t[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    t[:3, 3] = xyz
    return t


def _quaternion_matrix(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# Grasp point to wrist camera optical frame, same numbers as mmo500_harvest_gripper.urdf.xacro
_TCP_TO_OPTICAL = (np.linalg.inv(_transform((0.265, 0.0, 0.09), (0.0, math.pi / 2, 0.0)))
                   @ _transform((0.08, 0.0, 0.045), (math.pi, -0.1, 0.0))
                   @ _transform((0.0, 0.0, 0.0), (-math.pi / 2, 0.0, -math.pi / 2)))


class HarvestNode(Node):
    """Arm pose, head controllers and harvest score of the MMO-500 harvester."""

    def __init__(self):
        super().__init__("hal_harvest_node")
        self.tcp = None
        self.force = 0.0
        self.score = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Robpose, "/mmo500/Robpose", self.__pose_cb, 10)
        self.create_subscription(Float32, "/tomato_greenhouse/grip_force", self.__force_cb, 10)
        self.create_subscription(String, "/tomato_greenhouse/score", self.__score_cb, latched)
        self.clients = {
            name: ActionClient(self, FollowJointTrajectory, f"/mmo500/{name}/follow_joint_trajectory")
            for name in ("arm_controller", "gripper_controller", "twist_controller")
        }

    def __pose_cb(self, msg):
        self.tcp = msg

    def __force_cb(self, msg):
        self.force = msg.data

    def __score_cb(self, msg):
        self.score = json.loads(msg.data)

    def follow(self, controller, joints, positions, duration):
        """Send one trajectory point and wait until the controller has reached it."""
        client = self.clients[controller]
        if not client.wait_for_server(timeout_sec=10.0):
            print(f"{controller} not available", flush=True)
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        goal.trajectory.points = [point]
        sent = client.send_goal_async(goal)
        if not _wait(sent, 10.0) or not sent.result().accepted:
            return False
        return _wait(sent.result().get_result_async(), duration + 15.0)


def _wait(future, timeout):
    end = time.time() + timeout
    while not future.done() and time.time() < end:
        time.sleep(0.01)
    return future.done()


# Mutes exceptions
def custom_thread_excepthook(args):
    if "spin" in args.thread.name:
        return
    sys.__excepthook__(args.exc_type, args.exc_value, args.exc_traceback)


threading.excepthook = custom_thread_excepthook

print("HAL MMO-500 harvester initializing", flush=True)
if not rclpy.ok():
    rclpy.init(args=None)

### HAL INIT ###
motor_node = MotorsNode("/mmo500/cmd_vel", MAX_V, MAX_W)
odometry_node = OdometryNode("/mmo500/odom")
front_laser_node = LaserNode("/mmo500/front_laser/scan")
back_laser_node = LaserNode("/mmo500/back_laser/scan")
camera_node = CameraNode("/mmo500/harvest_camera/image")
depth_node = DepthCameraNode("/mmo500/harvest_camera/depth_image", "/mmo500/harvest_camera/camera_info")
harvest_node = HarvestNode()
arm = RBT(suffix="_mmo500", robmove_action="/mmo500/Robmove", use_move=False)

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
for node in (odometry_node, front_laser_node, back_laser_node, camera_node, depth_node, harvest_node):
    executor.add_node(node)


def __auto_spin() -> None:
    while rclpy.ok():
        try:
            executor.spin_once(timeout_sec=0)
        except Exception:
            pass
        time.sleep(1 / freq)


executor_thread = threading.Thread(target=__auto_spin, daemon=True)
executor_thread.start()


### BASE ###


def getPose3d():
    return odometry_node.getPose3d()


def getFrontLaserData():
    laser_data = front_laser_node.getLaserData()
    while len(laser_data.values) == 0:
        laser_data = front_laser_node.getLaserData()
    return laser_data


def getBackLaserData():
    laser_data = back_laser_node.getLaserData()
    while len(laser_data.values) == 0:
        laser_data = back_laser_node.getLaserData()
    return laser_data


def setV(velocity):
    motor_node.sendV(max(-MAX_V, min(MAX_V, float(velocity))))


def setVY(velocity):
    """Sideways speed of the mecanum base in m/s, positive to the left."""
    motor_node.last_twist.linear.y = max(-MAX_V, min(MAX_V, float(velocity)))
    motor_node.pub.publish(motor_node.last_twist)


def setW(velocity):
    motor_node.sendW(max(-MAX_W, min(MAX_W, float(velocity))))


### ARM ###


def moveArm(x, y, z, heading=-math.pi / 2, speed=0.3, motion="PTP"):
    """Move the grasp point between the fingers to x, y, z in m relative to the base.

    The head stays level with its fingers pointing towards heading, an angle around
    the vertical where -pi/2 points to the right of the robot. motion is PTP or LIN.
    """
    c, s = math.cos(heading), math.sin(heading)
    # Grasp frame with z along the fingers and x up
    m = np.array([[0.0, s, c], [0.0, -c, s], [1.0, 0.0, 0.0]])
    qw = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    qx = math.copysign(math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    qy = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    qz = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    pose = Robpose()
    pose.x, pose.y, pose.z = float(x), float(y), float(z)
    pose.qx, pose.qy, pose.qz, pose.qw = qx, qy, qz, qw
    result = arm.RobMove_EXECUTE(motion, float(speed), pose)
    if not result["Success"]:
        print("Arm movement FAILED, check the MoveIt output", flush=True)
    return bool(result["Success"])


def getArmPose():
    """Grasp point as [x, y, z, heading] relative to the base, None until the first reading."""
    tcp = harvest_node.tcp
    if tcp is None:
        return None
    axis = _quaternion_matrix(tcp.qx, tcp.qy, tcp.qz, tcp.qw)[:, 2]
    return [tcp.x, tcp.y, tcp.z, math.atan2(axis[1], axis[0])]


def homeArm(duration=4.0):
    return harvest_node.follow("arm_controller", ARM_JOINTS, ARM_HOME, duration)


### HARVEST HEAD ###


def setGripper(angle, duration=0.4):
    """Close the three fingers to angle in rad, from 0.0 open to 0.20 fully closed."""
    angle = max(GRIPPER_OPEN, min(GRIPPER_CLOSED, float(angle)))
    return harvest_node.follow("gripper_controller", ["harvest_finger_1_joint"], [angle], duration)


def openGripper():
    return setGripper(GRIPPER_OPEN)


def getGripForce():
    """Force in N the fingers squeeze the held tomato with."""
    return harvest_node.force


def closeGripperUntil(force, step=0.005):
    """Close the fingers little by little until the grip reaches force in N and return the force reached."""
    angle = GRIPPER_OPEN
    while angle < GRIPPER_CLOSED and getGripForce() < force:
        angle = min(GRIPPER_CLOSED, angle + step)
        setGripper(angle, 0.15)
        time.sleep(0.15)
    return getGripForce()


def twistGripper(angle, duration=1.0):
    """Turn the fingers around their shaft to angle in rad, between -pi and pi."""
    angle = max(-math.pi, min(math.pi, float(angle)))
    return harvest_node.follow("twist_controller", ["harvest_twist_joint"], [angle], duration)


### WRIST CAMERA ###


def getImage():
    image = camera_node.getImage()
    while image is None:
        image = camera_node.getImage()
    return image.data


def getDepthImage():
    """Depth of every pixel in m, inf where nothing is in range."""
    while depth_node.depth is None:
        time.sleep(0.01)
    return depth_node.depth.copy()


def pixelToPoint(u, v):
    """Point seen at pixel column u and row v as [x, y, z] in m relative to the base.

    Returns None when that pixel has no depth or the arm pose is still unknown.
    """
    depth, k, tcp = depth_node.depth, depth_node.k, harvest_node.tcp
    if depth is None or k is None or tcp is None:
        return None
    u, v = int(round(u)), int(round(v))
    patch = depth[max(0, v - 2):v + 3, max(0, u - 2):u + 3]
    patch = patch[np.isfinite(patch)]
    if patch.size == 0:
        return None
    z = float(np.median(patch))
    point = np.array([(u - k[2]) * z / k[0], (v - k[5]) * z / k[4], z, 1.0])
    base_to_tcp = np.eye(4)
    base_to_tcp[:3, :3] = _quaternion_matrix(tcp.qx, tcp.qy, tcp.qz, tcp.qw)
    base_to_tcp[:3, 3] = [tcp.x, tcp.y, tcp.z]
    return list((base_to_tcp @ _TCP_TO_OPTICAL @ point)[:3])


### SCORE ###


def getScore():
    """Harvested, green_in_crate, crushed, dropped and broken_trusses counters."""
    return dict(harvest_node.score)
