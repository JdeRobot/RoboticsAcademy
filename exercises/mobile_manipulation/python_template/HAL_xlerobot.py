import os
import subprocess
import sys
import threading
import time
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import (
    OdometryNode,
    quat2Yaw,
    quat2Pitch,
    quat2Roll,
)
from hal_interfaces.general.sim_time import SimTimeNode
from hal_interfaces.general.camera import CameraNode
from hal_interfaces.general.depth_camera import DepthCameraNode

# RBT client of ros2srrc_execution as in pick_place
sys.path.append(
    os.path.join(get_package_share_directory("ros2srrc_execution"), "python", "robot")
)
from robot import RBT  # noqa: E402
from ros2srrc_data.msg import Robpose  # noqa: E402

# HAL for XLeRobot in the warehouse exercise
# Cartesian arm goals are planned with MoveIt2 and are relative to base_footprint
# so they stay valid while the robot drives
# The joint functions move the arms directly without planning
# Objects are located with getImage then pixelToCameraPoint then cameraToBase

LEFT_ARM_JOINTS = ["Rotation_L", "Pitch_L", "Elbow_L", "Wrist_Pitch_L", "Wrist_Roll_L"]
RIGHT_ARM_JOINTS = ["Rotation_R", "Pitch_R", "Elbow_R", "Wrist_Pitch_R", "Wrist_Roll_R"]

# Jaw joint limits are the mechanical stops
GRIPPER_OPEN = 1.7453292
GRIPPER_CLOSED = -0.374533
# Jaw angle that holds the cube without pushing it out
GRASP_JAW = 0.55
# Jaw angle used to release and around the tray
GRASP_OPEN = 1.0
# Wider jaw angle while approaching an object so it enters the gap cleanly
GRASP_APPROACH_OPEN = 1.4

# Fully extended zero pose
ARM_HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0]

# Joint poses that place each gripper over its tray spot pointing down
LEFT_TRAY_JOINTS = [-1.9181, 1.1113, 2.0946, 0.7083, -1.5708]
RIGHT_TRAY_JOINTS = [2.116, 1.1136, 2.099, 0.7062, -1.5708]

# Both arms forward with the grippers pointing down
READY_LEFT_JOINTS = [-1.600, 1.527, 2.123, 0.063, -1.571]
READY_RIGHT_JOINTS = [1.630, 1.525, 2.110, 0.074, -1.571]


def custom_thread_excepthook(args):
    if "spin" in args.thread.name:
        return
    sys.__excepthook__(args.exc_type, args.exc_value, args.exc_traceback)


threading.excepthook = custom_thread_excepthook

print("HAL (XLeRobot Warehouse) initializing", flush=True)
if not rclpy.ok():
    rclpy.init(args=None)

### HAL INIT ###
motor_node = MotorsNode("/logistic_robot/cmd_vel", 4, 0.3)
odometry_node = OdometryNode("/logistic_robot/odom")
sim_time_node = SimTimeNode()

# Cameras
head_camera_node = CameraNode("/logistic_robot/head_camera/image")
left_arm_camera_node = CameraNode("/logistic_robot/left_arm_camera/image_raw")
right_arm_camera_node = CameraNode("/logistic_robot/right_arm_camera/image_raw")


class JointStateNode(Node):
    """Latest position and velocity of every joint by name."""

    def __init__(self, topic):
        super().__init__("hal_joint_state_node")
        self.positions = {}
        self.velocities = {}
        self.create_subscription(JointState, topic, self.__callback, 10)

    def __callback(self, msg):
        for i, name in enumerate(msg.name):
            self.positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.velocities[name] = msg.velocity[i]


joint_state_node = JointStateNode("/logistic_robot/joint_states")

head_depth_node = DepthCameraNode(
    "/logistic_robot/head_camera/depth_image", "/logistic_robot/head_camera/camera_info"
)

# Grippers and joint level arm control
arm_node = Node("hal_arm_node")

# Read by the gz_link_attacher plugins of the world
# Each gripper has its own attach switch and a stow switch that fixes the
# held object to the robot tray
attach_pubs = {
    side: arm_node.create_publisher(Bool, f"/xlerobot/{side}_gripper_attach", 10)
    for side in ("left", "right")
}
stow_pubs = {
    side: arm_node.create_publisher(Bool, f"/xlerobot/{side}_gripper_stow", 10)
    for side in ("left", "right")
}
graspable_pub = arm_node.create_publisher(String, "/graspable_objects", 10)

# The TF buffer has its own node because sharing arm_node left it with stale transforms
tf_node = Node("hal_tf_node")
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, tf_node)


def __joint_client(controller_name):
    client = ActionClient(
        arm_node,
        FollowJointTrajectory,
        f"/logistic_robot/{controller_name}/follow_joint_trajectory",
    )
    print(f"[HAL] waiting for {controller_name}...", flush=True)
    while not client.wait_for_server(timeout_sec=1.0):
        print(f"[HAL] still waiting for {controller_name}...", flush=True)
    print(f"[HAL] {controller_name} ready", flush=True)
    return client


left_arm_joint_client = __joint_client("left_arm_controller")
right_arm_joint_client = __joint_client("right_arm_controller")
left_gripper_client = __joint_client("left_gripper_controller")
right_gripper_client = __joint_client("right_gripper_controller")
head_joint_client = __joint_client("head_controller")


def __send_trajectory(client, joint_names, positions, duration):
    goal_msg = FollowJointTrajectory.Goal()
    goal_msg.trajectory.joint_names = joint_names

    point = JointTrajectoryPoint()
    point.positions = [float(p) for p in positions]
    whole_secs = int(duration)
    point.time_from_start = Duration(
        sec=whole_secs, nanosec=int((duration - whole_secs) * 1e9)
    )

    goal_msg.trajectory.points.append(point)
    client.send_goal_async(goal_msg)


# Gripper pose of each arm
class ArmPose:
    """Gripper pose in meters and radians shaped like the odometry Pose3d."""

    def __init__(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.qx = msg.qx
        self.qy = msg.qy
        self.qz = msg.qz
        self.qw = msg.qw
        self.yaw = quat2Yaw(msg.qw, msg.qx, msg.qy, msg.qz)
        self.pitch = quat2Pitch(msg.qw, msg.qx, msg.qy, msg.qz)
        self.roll = quat2Roll(msg.qw, msg.qx, msg.qy, msg.qz)

    def __str__(self):
        return (
            f"ArmPose(x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f}, "
            f"roll={self.roll:.2f}, pitch={self.pitch:.2f}, yaw={self.yaw:.2f})"
        )


class ArmPoseNode(Node):
    def __init__(self, node_name, topic):
        super().__init__(node_name)
        self.pose = None
        self.create_subscription(Robpose, topic, self.__callback, 10)

    def __callback(self, msg):
        self.pose = ArmPose(msg)


left_pose_node = ArmPoseNode(
    "hal_left_pose_node", "/logistic_robot/left_robpose/Robpose"
)
right_pose_node = ArmPoseNode(
    "hal_right_pose_node", "/logistic_robot/right_robpose/Robpose"
)

executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(odometry_node)
executor.add_node(sim_time_node)
executor.add_node(arm_node)
executor.add_node(left_pose_node)
executor.add_node(right_pose_node)
executor.add_node(head_camera_node)
executor.add_node(left_arm_camera_node)
executor.add_node(right_arm_camera_node)
executor.add_node(head_depth_node)
executor.add_node(joint_state_node)
executor.add_node(tf_node)


def __auto_spin() -> None:
    # A continuous spin keeps TF and the other callbacks from falling behind
    try:
        executor.spin()
    except Exception:
        pass


executor_thread = threading.Thread(target=__auto_spin, daemon=True)
executor_thread.start()


def __publish_graspable_objects():
    msg = String()
    msg.data = "blue_box,yellow_box,green_cylinder"
    graspable_pub.publish(msg)


arm_node.create_timer(1.0, __publish_graspable_objects)


# One Robmove client per arm
print("[HAL] connecting to left arm Robmove action...", flush=True)
LEFT_ARM = RBT(
    suffix="_left",
    robmove_action="/logistic_robot/left_robmove/Robmove",
    use_move=False,
)
print("[HAL] connecting to right arm Robmove action...", flush=True)
RIGHT_ARM = RBT(
    suffix="_right",
    robmove_action="/logistic_robot/right_robmove/Robmove",
    use_move=False,
)


def __quaternion_from_rpy(roll, pitch, yaw):
    """Quaternion from roll pitch and yaw in radians."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw


def __move_cartesian(arm, x, y, z, roll, pitch, yaw, speed, motion, side):
    qx, qy, qz, qw = __quaternion_from_rpy(roll, pitch, yaw)

    target = Robpose()
    target.x = float(x)
    target.y = float(y)
    target.z = float(z)
    target.qx = qx
    target.qy = qy
    target.qz = qz
    target.qw = qw

    print(
        f"[HAL] {side} arm {motion} to xyz=({x:.3f}, {y:.3f}, {z:.3f}) "
        f"rpy=({roll:.2f}, {pitch:.2f}, {yaw:.2f})",
        flush=True,
    )

    result = arm.RobMove_EXECUTE(motion, float(speed), target)

    if result["Success"]:
        print(f"[HAL] {side} arm move done in {result['ExecTime']} s", flush=True)
    else:
        print(f"[HAL] {side} arm move FAILED: {result['Message']}", flush=True)

    return result["Success"]


def getPose3d():
    """Base pose from odometry."""
    return odometry_node.getPose3d()


def worldToBase(x, y, z):
    """Warehouse point expressed in base_footprint using odometry."""
    pose = odometry_node.getPose3d()
    dx = x - pose.x
    dy = y - pose.y
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return (c * dx + s * dy, -s * dx + c * dy, z)


def getJointPositions():
    """Last reported position in radians of every joint by name."""
    return dict(joint_state_node.positions)


def getSimTime():
    return sim_time_node.getSimTime()


def __sim_now():
    t = sim_time_node.getSimTime()
    return t.seconds + t.nanoseconds * 1e-9


def sleepSim(seconds):
    """Wait in simulation time because the simulation runs slower than real time."""
    start = __sim_now()
    deadline = time.time() + seconds * 20 + 5
    while __sim_now() - start < seconds and time.time() < deadline:
        time.sleep(0.02)


def getImage():
    """Head camera image in BGR."""
    image = head_camera_node.getImage()
    while image is None:
        image = head_camera_node.getImage()
    return image.data


def getLeftArmImage():
    """Left wrist camera image in BGR."""
    image = left_arm_camera_node.getImage()
    while image is None:
        image = left_arm_camera_node.getImage()
    return image.data


def getRightArmImage():
    """Right wrist camera image in BGR."""
    image = right_arm_camera_node.getImage()
    while image is None:
        image = right_arm_camera_node.getImage()
    return image.data


# Gazebo cameras cannot be disabled so off means a very low render rate
CAMERA_SET_RATE_SERVICES = {
    "head": "/logistic_robot/head_camera/set_rate",
    "left_arm": "/logistic_robot/left_arm_camera/image_raw/set_rate",
    "right_arm": "/logistic_robot/right_arm_camera/image_raw/set_rate",
}
CAMERA_ON_HZ = 15.0  # same as update_rate in xlerobot_gz.urdf.xacro
CAMERA_IDLE_HZ = 0.2


def setCameraRate(camera, hz):
    """Set the render rate in Hz of one camera and return True if Gazebo accepted it."""
    service = CAMERA_SET_RATE_SERVICES[camera]
    try:
        # gz service returns 0 even when the service does not exist
        info = subprocess.run(
            ["gz", "service", "-i", "-s", service],
            capture_output=True,
            text=True,
            timeout=10,
        )
        if "Service providers" not in info.stdout:
            print(f"[HAL] setCameraRate: no Gazebo service {service}", flush=True)
            return False
        subprocess.run(
            [
                "gz",
                "service",
                "-s",
                service,
                "--reqtype",
                "gz.msgs.Double",
                "--reptype",
                "gz.msgs.Empty",
                "--timeout",
                "3000",
                "--req",
                f"data: {float(hz)}",
            ],
            capture_output=True,
            text=True,
            timeout=10,
            check=True,
        )
    except (OSError, subprocess.SubprocessError) as e:
        print(f"[HAL] setCameraRate({camera}, {hz}) failed: {e}", flush=True)
        return False
    return True


def cameraOff(*cameras):
    """Slow down the given cameras or all of them when none is given."""
    return all(
        setCameraRate(c, CAMERA_IDLE_HZ) for c in (cameras or CAMERA_SET_RATE_SERVICES)
    )


def cameraOn(*cameras):
    """Restore the render rate. The first new frame can take a few seconds."""
    return all(
        setCameraRate(c, CAMERA_ON_HZ) for c in (cameras or CAMERA_SET_RATE_SERVICES)
    )


def getDepthImage():
    """Head camera depth in meters on the same pixel grid as getImage."""
    while head_depth_node.depth is None:
        pass
    return head_depth_node.depth


def pixelToCameraPoint(u, v):
    """3D point in the head camera optical frame for a pixel or None without valid depth."""
    depth = head_depth_node.depth
    k = head_depth_node.k
    if depth is None or k is None:
        return None
    d = float(depth[int(v), int(u)])
    if not math.isfinite(d) or d <= 0.0:
        return None
    fx, cx, fy, cy = k[0], k[2], k[4], k[5]
    x = (u - cx) * d / fx
    y = (v - cy) * d / fy
    return (x, y, d)


def __quat_to_matrix(qx, qy, qz, qw):
    return np.array(
        [
            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx**2 + qy**2)],
        ]
    )


def cameraToBase(x, y, z):
    """Point in base_footprint from the head camera optical frame or None if TF is not ready."""
    if head_depth_node.depth is None:
        return None
    try:
        # The gz frame_id is not a TF frame, so the URDF link name is used
        t = tf_buffer.lookup_transform(
            "base_footprint", "head_camera_depth_frame", rclpy.time.Time()
        )
    except Exception as e:
        print(f"[HAL] cameraToBase TF lookup failed: {e}", flush=True)
        return None

    r = t.transform.rotation
    tr = t.transform.translation
    rot = __quat_to_matrix(r.x, r.y, r.z, r.w)
    point = rot @ np.array([x, y, z]) + np.array([tr.x, tr.y, tr.z])
    return tuple(point)


# The base reaches the commanded speeds with limited acceleration
# because a sudden speed change swings the arms around their weak joints
BASE_MAX_LINEAR_ACCEL = 0.4  # m/s^2
BASE_MAX_ANGULAR_ACCEL = 0.5  # rad/s^2
_base_target = [0.0, 0.0, 0.0]
_base_current = [0.0, 0.0, 0.0]


def __base_ramp():
    last = __sim_now()
    while True:
        time.sleep(0.01)
        now = __sim_now()
        dt = now - last
        if dt <= 0.0:
            continue
        last = now
        limits = (
            BASE_MAX_LINEAR_ACCEL * dt,
            BASE_MAX_LINEAR_ACCEL * dt,
            BASE_MAX_ANGULAR_ACCEL * dt,
        )
        changed = False
        for i, step in enumerate(limits):
            delta = max(-step, min(step, _base_target[i] - _base_current[i]))
            if delta != 0.0:
                _base_current[i] += delta
                changed = True
        if changed:
            twist = motor_node.last_twist
            twist.linear.x = _base_current[0]
            twist.linear.y = _base_current[1]
            twist.angular.z = _base_current[2]
            motor_node.pub.publish(twist)


def setV(velocity):
    """Forward speed in m/s."""
    _base_target[0] = float(velocity)


def setVY(velocity):
    """Sideways speed in m/s."""
    _base_target[1] = float(velocity)


def setW(velocity):
    """Yaw rate in rad/s."""
    _base_target[2] = float(velocity)


threading.Thread(target=__base_ramp, daemon=True).start()


def moveLeftArm(x, y, z, roll=0.0, pitch=0.0, yaw=0.0, speed=0.3, motion="PTP"):
    """Move the left gripper to a pose relative to base_footprint.
    motion is PTP for a free move or LIN for a straight line.
    Blocks until the move ends and returns True on success.
    """
    return __move_cartesian(LEFT_ARM, x, y, z, roll, pitch, yaw, speed, motion, "left")


def moveRightArm(x, y, z, roll=0.0, pitch=0.0, yaw=0.0, speed=0.3, motion="PTP"):
    """Move the right gripper like moveLeftArm."""
    return __move_cartesian(
        RIGHT_ARM, x, y, z, roll, pitch, yaw, speed, motion, "right"
    )


def getLeftArmPose():
    """Current left gripper pose or None until the first Robpose message."""
    return left_pose_node.pose


def getRightArmPose():
    """Current right gripper pose or None until the first Robpose message."""
    return right_pose_node.pose


def setLeftArmJoints(positions, duration=2.0):
    """Move the left arm joints directly without planning or collision checks.
    positions follows the order of LEFT_ARM_JOINTS in radians.
    """
    __send_trajectory(left_arm_joint_client, LEFT_ARM_JOINTS, positions, duration)


def setRightArmJoints(positions, duration=2.0):
    """Move the right arm joints directly in the order of RIGHT_ARM_JOINTS."""
    __send_trajectory(right_arm_joint_client, RIGHT_ARM_JOINTS, positions, duration)


def setHeadJoints(pan, tilt, duration=1.0):
    """Move the head joints directly. Tilt goes from 0.76 up to 1.45 down."""
    __send_trajectory(
        head_joint_client, ["head_pan_joint", "head_tilt_joint"], [pan, tilt], duration
    )


def homeLeftArm(duration=2.0):
    """Send the left arm to its fully extended zero pose."""
    setLeftArmJoints(ARM_HOME_JOINTS, duration)


def homeRightArm(duration=2.0):
    """Send the right arm to its fully extended zero pose."""
    setRightArmJoints(ARM_HOME_JOINTS, duration)


def leftTrayPose(duration=2.0):
    """Left arm to its tray spot."""
    setLeftArmJoints(LEFT_TRAY_JOINTS, duration)


def rightTrayPose(duration=2.0):
    """Right arm to its tray spot."""
    setRightArmJoints(RIGHT_TRAY_JOINTS, duration)


def readyPose(duration=2.0):
    """Both arms to the ready stance."""
    setLeftArmJoints(READY_LEFT_JOINTS, duration)
    setRightArmJoints(READY_RIGHT_JOINTS, duration)


def setLeftGripper(closed, duration=1.0, target=None):
    """True closes the left gripper and arms auto attach and False opens it.
    target overrides the jaw angle.
    """
    attach_pubs["left"].publish(Bool(data=bool(closed)))
    if target is None:
        target = GRIPPER_CLOSED if closed else GRIPPER_OPEN
    __send_trajectory(left_gripper_client, ["Jaw_L"], [target], duration)


def setRightGripper(closed, duration=1.0, target=None):
    """Right gripper like setLeftGripper."""
    attach_pubs["right"].publish(Bool(data=bool(closed)))
    if target is None:
        target = GRIPPER_CLOSED if closed else GRIPPER_OPEN
    __send_trajectory(right_gripper_client, ["Jaw_R"], [target], duration)


# The object is fixed to its tray spot so a later pick finds it again
def dropInLeftTray(speed=0.3):
    """Stow the held cube in the left arm's tray spot."""
    return __tray_visit("left", True, speed)


def dropInRightTray(speed=0.3):
    """Stow the held cube in the right arm's tray spot."""
    return __tray_visit("right", True, speed)


def pickFromLeftTray(speed=0.3):
    """Take the cube left by dropInLeftTray."""
    return __tray_visit("left", False, speed)


def pickFromRightTray(speed=0.3):
    """Take the cube left by dropInRightTray."""
    return __tray_visit("right", False, speed)


# The arms have 5 joints so MoveIt only accepts exactly reachable poses
# The grasp orientation is solved numerically so every pose is exactly reachable
GRASP_PITCH = 0.4
GRASP_APPROACH_DISTANCE = 0.10

# Arm kinematic chain from the xacro with origin xyz and rpy and axis of each joint
_ARM_CHAIN = [
    ((0, -0.0452, 0.0165), (1.5708, 0, 0), (0, -1, 0)),
    ((0, 0.1025, 0.0306), (1.5708, 0, 0), (-1, 0, 0)),
    ((0, 0.11257, 0.028), (-1.5708, 0, 0), (1, 0, 0)),
    ((0, 0.0052, 0.1349), (-1.5708, 0, 0), (1, 0, 0)),
    ((0, -0.0601, 0), (0, 1.5707927, 0), (0, -1, 0)),
]
_TCP_OFFSET = (-0.0380, -0.0930, 0.0)
_ARM_LOWER = np.array([-2.16, -0.22, -0.22, -1.6580628, -2.7438473])
_ARM_UPPER = np.array([2.16, 3.37, 3.14, 1.6580627, 2.8412063])
# Arm mount position and yaw in base_footprint
_ARM_MOUNT = {
    "left": (np.array([-0.09, -0.11, 0.765]), 0.0),
    "right": (np.array([-0.09, 0.11, 0.765]), math.pi),
}
_ARM_SEEDS = {
    "left": [
        [-1.6, 1.5, 2.1, 0.06, 1.571],
        [-1.6, 1.5, 2.1, 0.06, -1.571],
        [-1.0, 1.2, 1.8, 0.5, 1.0],
        [-1.0, 1.2, 1.8, 0.5, -1.0],
        [0.0, 1.0, 1.5, 0.3, 0.0],
    ],
    "right": [
        [1.63, 1.52, 2.11, 0.07, 1.571],
        [1.63, 1.52, 2.11, 0.07, -1.571],
        [1.0, 1.2, 1.8, 0.5, 1.0],
        [1.0, 1.2, 1.8, 0.5, -1.0],
        [0.0, 1.0, 1.5, 0.3, 0.0],
    ],
}


def __rot(axis, angle):
    a = np.array(axis, dtype=float)
    a /= np.linalg.norm(a)
    x, y, z = a
    c, s = math.cos(angle), math.sin(angle)
    C = 1 - c
    return np.array(
        [
            [x * x * C + c, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, z * z * C + c],
        ]
    )


def __rpy_matrix(roll, pitch, yaw):
    return __rot((0, 0, 1), yaw) @ __rot((0, 1, 0), pitch) @ __rot((1, 0, 0), roll)


def __arm_fk(arm, joints):
    """TCP position and rotation in base_footprint for the given joint angles."""
    base, mount_yaw = _ARM_MOUNT[arm]
    R = np.eye(3)
    p = np.zeros(3)
    for (xyz, rpy, axis), q in zip(_ARM_CHAIN, joints):
        p = p + R @ np.array(xyz)
        R = R @ __rpy_matrix(*rpy) @ __rot(axis, q)
    p = p + R @ np.array(_TCP_OFFSET)
    Rm = __rot((0, 0, 1), mount_yaw)
    return Rm @ p + base, Rm @ R


def __solve_grasp_orientation(arm, target, pitch=None, roll_sign=None):
    """Rotation and finger direction for a grasp at target or None if unreachable."""
    from scipy.optimize import least_squares

    target = np.array(target, dtype=float)
    pitch = GRASP_PITCH if pitch is None else pitch

    def residual(q):
        p, R = __arm_fk(arm, q)
        f = -R[:, 1]  # fingers point along negative TCP y
        horizontal = np.array([f[0], f[1], 0.0])
        horizontal /= np.linalg.norm(horizontal)
        sideways = np.array([-horizontal[1], horizontal[0], 0.0])
        return np.concatenate(
            [(p - target) * 10.0, [math.asin(-f[2]) - pitch, R[:, 2] @ sideways]]
        )

    # roll_sign picks one of the two exact solutions that differ in wrist roll
    solutions = []
    for seed in _ARM_SEEDS[arm]:
        r = least_squares(
            residual,
            seed,
            bounds=(_ARM_LOWER, _ARM_UPPER),
            xtol=1e-12,
            ftol=1e-12,
            gtol=1e-12,
        )
        cost = float(np.linalg.norm(r.fun))
        if cost < 1e-6:
            solutions.append((cost, r.x))
    if not solutions:
        return None
    if roll_sign is not None:
        wanted = [s for s in solutions if s[1][4] * roll_sign > 0]
        solutions = wanted or solutions
    best = min(solutions, key=lambda s: s[0])
    _, R = __arm_fk(arm, best[1])
    return R, -R[:, 1], best[1]


def __solve_pose(arm, position, R, seed):
    """Joint angles that reach position with rotation R exactly or None."""
    from scipy.optimize import least_squares

    def residual(q):
        p, Rq = __arm_fk(arm, q)
        return np.concatenate([(p - position) * 10.0, (Rq - R).ravel()])

    r = least_squares(
        residual,
        seed,
        bounds=(_ARM_LOWER, _ARM_UPPER),
        xtol=1e-12,
        ftol=1e-12,
        gtol=1e-12,
    )
    return r.x if np.linalg.norm(r.fun) < 1e-6 else None


def __rpy_from_matrix(R):
    pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    return math.atan2(R[2, 1], R[2, 2]), pitch, math.atan2(R[1, 0], R[0, 0])


# The hand reaches the tray through a carry pose and a waypoint above the spot
# so the cube clears the front rim of the tray
# The spots are joint poses with the fingers nearly straight down
TRAY_DROP_JOINTS = {
    "left": [-1.9181, 1.276, 2.4177, 0.3084, -1.5708],
    "right": [2.116, 1.1136, 2.099, 0.7062, -1.5708],
}
TRAY_ABOVE_JOINTS = {
    "left": [-1.9181, 1.928, 2.1165, 1.2615, -1.5708],
    "right": [2.116, 1.6408, 1.8219, 1.5106, -1.5708],
}
TRAY_CARRY_JOINTS = {
    "left": [-1.544, 1.356, 1.66, 0.612, -1.571],
    "right": [1.86, 1.332, 1.621, 0.142, -1.571],
}


# Grasping placing and the tray run joint paths from the exact solver of this HAL
# because MoveIt IK is unreliable on a 5 joint arm and can pick a flipped branch
JOINT_SPEED = 1.0  # rad/s at the default speed
LINE_SPEED = 0.05  # m/s at the default speed
LINE_STEPS = 20


def __arm_names(arm):
    return LEFT_ARM_JOINTS if arm == "left" else RIGHT_ARM_JOINTS


def __set_gripper(arm, closed, target):
    (setLeftGripper if arm == "left" else setRightGripper)(closed, 1.0, target)
    sleepSim(1.0)


def __run_path(arm, path, step_times):
    """Send a joint path to the arm controller and wait until it ends."""
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = __arm_names(arm)
    t = 0.0
    for q, dt in zip(path, step_times):
        t += dt
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in q]
        whole = int(t)
        point.time_from_start = Duration(sec=whole, nanosec=int((t - whole) * 1e9))
        goal.trajectory.points.append(point)
    client = left_arm_joint_client if arm == "left" else right_arm_joint_client
    client.send_goal_async(goal)
    sleepSim(t + 0.5)


def __joint_move(arm, q, speed):
    current = joint_state_node.positions
    q0 = np.array([current.get(n, 0.0) for n in __arm_names(arm)])
    rate = JOINT_SPEED * speed / 0.3
    __run_path(arm, [q], [max(1.0, float(np.max(np.abs(np.array(q) - q0))) / rate)])


def __line_move(arm, start, end, R, seed, speed):
    """Straight TCP line with fixed orientation. Returns the final joints or None."""
    start, end = np.array(start, dtype=float), np.array(end, dtype=float)
    path = []
    q = np.array(seed, dtype=float)
    for k in range(1, LINE_STEPS + 1):
        q_next = __solve_pose(arm, start + (end - start) * k / LINE_STEPS, R, q)
        if q_next is None or np.max(np.abs(q_next - q)) > 0.3:
            print(f"[HAL] {arm} straight line is not reachable", flush=True)
            return None
        q = q_next
        path.append(q)
    step = np.linalg.norm(end - start) / LINE_STEPS / (LINE_SPEED * speed / 0.3)
    __run_path(arm, path, [max(0.05, step)] * LINE_STEPS)
    return q


def __approach(arm, x, y, z):
    """Grasp rotation and joints at the point and at the approach point or None."""
    # Same wrist roll as the tray and ready poses so the wrist never flips
    solved = __solve_grasp_orientation(arm, (x, y, z), roll_sign=-1.0)
    if solved is None:
        print(f"[HAL] {arm} arm cannot reach ({x:.3f}, {y:.3f}, {z:.3f})", flush=True)
        return None
    R, f, q = solved
    pre = np.array((x, y, z)) - f * GRASP_APPROACH_DISTANCE
    q_pre = __solve_pose(arm, pre, R, q)
    if q_pre is None:
        print(f"[HAL] {arm} arm cannot reach the approach point", flush=True)
        return None
    return R, pre, q, q_pre


def __grasp(arm, x, y, z, speed):
    solved = __approach(arm, x, y, z)
    if solved is None:
        return False
    R, pre, q, q_pre = solved
    __set_gripper(arm, False, GRASP_APPROACH_OPEN)
    __joint_move(arm, q_pre, speed)
    if __line_move(arm, pre, (x, y, z), R, q_pre, speed) is None:
        return False
    __set_gripper(arm, True, GRASP_JAW)
    if __line_move(arm, (x, y, z), pre, R, q, speed) is None:
        __joint_move(arm, q_pre, speed)
    return True


def __place(arm, x, y, z, speed):
    solved = __approach(arm, x, y, z)
    if solved is None:
        return False
    R, pre, q, q_pre = solved
    # Leave the robot tray through the carry pose so the object clears the rim
    __joint_move(arm, TRAY_CARRY_JOINTS[arm], speed)
    __joint_move(arm, q_pre, speed)
    if __line_move(arm, pre, (x, y, z), R, q_pre, speed) is None:
        return False
    __set_gripper(arm, False, GRASP_OPEN)
    if __line_move(arm, (x, y, z), pre, R, q, speed) is None:
        __joint_move(arm, q_pre, speed)
    return True


def __tray_visit(arm, release, speed):
    """Go over the tray spot and lower the hand to release or take the object."""
    q_drop = TRAY_DROP_JOINTS[arm]
    q_above = TRAY_ABOVE_JOINTS[arm]
    p_drop, R = __arm_fk(arm, q_drop)
    p_above, _ = __arm_fk(arm, q_above)
    if not release:
        __set_gripper(arm, False, GRASP_OPEN)
    __joint_move(arm, TRAY_CARRY_JOINTS[arm], speed)
    __joint_move(arm, q_above, speed)
    if __line_move(arm, p_above, p_drop, R, q_above, speed) is None:
        __joint_move(arm, q_drop, speed)
    if release:
        stow_pubs[arm].publish(Bool(data=True))
        sleepSim(0.5)
        __set_gripper(arm, False, GRASP_OPEN)
    else:
        __set_gripper(arm, True, GRASP_JAW)
    if __line_move(arm, p_drop, p_above, R, q_drop, speed) is None:
        __joint_move(arm, q_above, speed)
    return True


def graspLeft(x, y, z, speed=0.3):
    """Grasp with the left arm an object centered at a point in base_footprint.
    Returns True only if every step succeeded.
    """
    return __grasp("left", x, y, z, speed)


def graspRight(x, y, z, speed=0.3):
    """Grasp with the right arm like graspLeft."""
    return __grasp("right", x, y, z, speed)


def placeLeft(x, y, z, speed=0.3):
    """Release the object held by the left arm at a point in base_footprint.
    z is the height of the object center when it is released.
    """
    return __place("left", x, y, z, speed)


def placeRight(x, y, z, speed=0.3):
    """Release the object held by the right arm like placeLeft."""
    return __place("right", x, y, z, speed)
