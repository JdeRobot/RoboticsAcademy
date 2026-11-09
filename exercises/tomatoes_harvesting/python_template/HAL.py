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
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from hal_interfaces.general.camera import CameraNode
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
# Unfolded in front of the base, MoveIt sees the folded arm touching the cabinet and plans best from here
ARM_READY = [1.1428, -2.1233, 1.9619, -1.4094, -1.5708, -1.9988]
ARM_SPEED = 0.3
# Pilz plans the same move every time, the harvest head joint limits keep the arm on one configuration
ARM_PLANNER = "PTP"
# Finger angles of the harvest head, the student only deals with forces
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 0.20
# The fingers close smoothly at this rate while the grip force is watched
GRIPPER_STEP = 0.003
GRIP_RATE = 20.0
# The head spins a tomato several fast turns, the shaft has five turns of travel each way
TWIST_SPEED = 2 * 2 * math.pi
TWIST_RANGE = 10 * math.pi
# Depth is only rendered while asked for and dropped after this long without use
DEPTH_IDLE = 5.0
DEPTH_TIMEOUT = 3.0


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


class ArmPose:
    """Grasp point relative to the base and the direction its fingers point to."""

    def __init__(self, x, y, z, heading):
        self.x = x
        self.y = y
        self.z = z
        self.heading = heading

    def __str__(self):
        return f"ArmPose: {{x: {self.x:.3f}, y: {self.y:.3f}, z: {self.z:.3f}, heading: {self.heading:.3f}}}"


class HarvestNode(Node):
    """Arm pose and head controllers of the MMO-500 harvester."""

    def __init__(self):
        super().__init__("hal_harvest_node")
        self.tcp = None
        self.force = 0.0
        self.create_subscription(Robpose, "/mmo500/Robpose", self.__pose_cb, 10)
        self.create_subscription(Float32, "/tomato_greenhouse/grip_force", self.__force_cb, 10)
        self.finger_pub = self.create_publisher(JointTrajectory, "/mmo500/gripper_controller/joint_trajectory", 10)
        self._controllers = {
            name: ActionClient(self, FollowJointTrajectory, f"/mmo500/{name}/follow_joint_trajectory")
            for name in ("arm_controller", "gripper_controller", "twist_controller")
        }

    def __pose_cb(self, msg):
        self.tcp = msg

    def __force_cb(self, msg):
        self.force = msg.data

    def follow(self, controller, joints, positions, duration, wait=True):
        """Send one trajectory point and wait until the controller has reached it."""
        client = self._controllers[controller]
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
        if not wait:
            return True
        if not _wait(sent, 10.0) or not sent.result().accepted:
            return False
        done = sent.result().get_result_async()
        if not _wait(done, duration + 15.0):
            return False
        # An aborted goal also answers, only a reached one counts
        return done.result().result.error_code == FollowJointTrajectory.Result.SUCCESSFUL


class DepthOnDemand(Node):
    """Depth of the wrist camera in m, subscribed only while it is being asked for."""

    def __init__(self, image_topic, info_topic):
        super().__init__("hal_harvest_depth")
        self.k = None
        self._topic = image_topic
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._sub = None
        self._depth = None
        self._stamp = 0.0
        self._last_use = 0.0
        self.create_subscription(CameraInfo, info_topic, self.__info_cb, 10)
        self.create_timer(1.0, self.__drop_when_idle)

    def __info_cb(self, msg):
        self.k = msg.k

    def __depth_cb(self, msg):
        depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        with self._lock:
            self._depth, self._stamp = depth, time.monotonic()

    def __drop_when_idle(self):
        with self._lock:
            if self._sub is not None and time.monotonic() - self._last_use > DEPTH_IDLE:
                self.destroy_subscription(self._sub)
                self._sub = None

    def get(self):
        """Depth image taken after the call, None if none arrives in time."""
        asked = time.monotonic()
        with self._lock:
            self._last_use = asked
            if self._sub is None:
                self._sub = self.create_subscription(Image, self._topic, self.__depth_cb, 1)
        while time.monotonic() - asked < DEPTH_TIMEOUT:
            with self._lock:
                if self._depth is not None and self._stamp >= asked:
                    return self._depth
            time.sleep(0.01)
        return None


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
depth_node = DepthOnDemand("/mmo500/harvest_camera/depth_image", "/mmo500/harvest_camera/camera_info")
harvest_node = HarvestNode()
arm = RBT(suffix="_mmo500", robmove_action="/mmo500/Robmove", use_move=False)

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
for node in (odometry_node, front_laser_node, back_laser_node, camera_node, depth_node, harvest_node):
    executor.add_node(node)


def _serve_callbacks():
    # One failing callback would stop every sensor for good, so it is reported and serving goes on
    while rclpy.ok():
        try:
            executor.spin_once(timeout_sec=0.1)
        except Exception as error:
            print(f"HAL callback error: {error!r}", flush=True)


# Callbacks are served as they arrive, the grip watches a force that must never lag behind
executor_thread = threading.Thread(target=_serve_callbacks, name="hal_spin", daemon=True)
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


def moveArm(x, y, z, heading=-math.pi / 2):
    """Move the grasp point between the fingers to x, y, z in m relative to the base.

    The head stays level and its fingers point towards heading, an angle around the
    vertical measured like the base yaw. The default points to the right of the robot.
    Returns True once the arm is there.
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
    if arm.RobMove_EXECUTE(ARM_PLANNER, ARM_SPEED, pose)["Success"]:
        return True
    # A failed plan is tried again once from the unfolded pose
    harvest_node.follow("arm_controller", ARM_JOINTS, ARM_READY, 3.0)
    # MoveIt plans from the current state so the arm settles first
    time.sleep(2.0)
    if arm.RobMove_EXECUTE(ARM_PLANNER, ARM_SPEED, pose)["Success"]:
        return True
    print("Arm movement FAILED, the point may be out of reach", flush=True)
    return False


def getArmPose():
    """Current grasp point as an ArmPose, None until the arm reports its first pose."""
    tcp = harvest_node.tcp
    if tcp is None:
        return None
    axis = _quaternion_matrix(tcp.qx, tcp.qy, tcp.qz, tcp.qw)[:, 2]
    return ArmPose(tcp.x, tcp.y, tcp.z, math.atan2(axis[1], axis[0]))


def homeArm():
    """Fold the arm back to the pose the robot starts in."""
    return harvest_node.follow("arm_controller", ARM_JOINTS, ARM_HOME, 4.0)


### HARVEST HEAD ###


_finger_angle = GRIPPER_OPEN
_twist_angle = 0.0


def _set_fingers(angle, duration):
    global _finger_angle
    _finger_angle = angle
    return harvest_node.follow("gripper_controller", ["harvest_finger_1_joint"], [angle], duration)


def grip(force):
    """Close the fingers until they squeeze with force in N and return the force reached.

    Returns 0.0 when the fingers closed completely without touching anything.
    """
    global _finger_angle
    # Keeps closing from where the fingers are so a firmer grip never lets go first
    angle = _finger_angle
    while angle < GRIPPER_CLOSED and harvest_node.force < force:
        angle = min(GRIPPER_CLOSED, angle + GRIPPER_STEP)
        point = JointTrajectoryPoint(positions=[angle], time_from_start=Duration(nanosec=int(1e9 / GRIP_RATE)))
        harvest_node.finger_pub.publish(JointTrajectory(joint_names=["harvest_finger_1_joint"], points=[point]))
        time.sleep(1.0 / GRIP_RATE)
    _finger_angle = angle
    # The force follows the fingers with a short delay
    time.sleep(0.2)
    return harvest_node.force


def release():
    """Open the fingers, the shaft winds back to zero on its own meanwhile."""
    global _twist_angle
    opened = _set_fingers(GRIPPER_OPEN, 0.4)
    if _twist_angle != 0.0:
        harvest_node.follow("twist_controller", ["harvest_twist_joint"], [0.0], max(0.2, abs(_twist_angle) / TWIST_SPEED), wait=False)
        _twist_angle = 0.0
    return opened


def twist(turns):
    """Spin the fingers this many turns, negative turns spin the other way."""
    global _twist_angle
    target = max(-TWIST_RANGE, min(TWIST_RANGE, _twist_angle + 2 * math.pi * float(turns)))
    duration = max(0.2, abs(target - _twist_angle) / TWIST_SPEED)
    _twist_angle = target
    return harvest_node.follow("twist_controller", ["harvest_twist_joint"], [target], duration)


### WRIST CAMERA ###


def getImage():
    image = camera_node.getImage()
    while image is None:
        image = camera_node.getImage()
    return image.data


def getDepthImage():
    """Depth of every pixel in m, inf where nothing is in range."""
    depth = depth_node.get()
    while depth is None:
        depth = depth_node.get()
    return depth.copy()


def pixelToPoint(u, v):
    """Point seen at pixel column u and row v as [x, y, z] in m relative to the base.

    Returns None when that pixel has no depth or the arm pose is still unknown.
    """
    depth, k, tcp = depth_node.get(), depth_node.k, harvest_node.tcp
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
