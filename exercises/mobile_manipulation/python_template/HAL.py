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
from hal_interfaces.general.odometry import OdometryNode, quat2Yaw, quat2Pitch, quat2Roll
from hal_interfaces.general.sim_time import SimTimeNode
from hal_interfaces.general.camera import CameraNode
from hal_interfaces.general.depth_camera import DepthCameraNode

# RBT client of ros2srrc_execution, same import as pick_place
sys.path.append(
    os.path.join(get_package_share_directory("ros2srrc_execution"), "python", "robot")
)
from robot import RBT  # noqa: E402
from ros2srrc_data.msg import Robpose  # noqa: E402

# Hardware Abstraction Layer for XLeRobot in the house exercise
#
# Arm control levels
#   moveLeftArm and moveRightArm plan a Cartesian goal with MoveIt2
#   Targets are relative to base_footprint so they stay valid while the robot moves
#   setLeftArmJoints and setRightArmJoints send raw joint angles with no planning
#   setLeftGripper and setRightGripper open and close the gripper
#
# Finding an object
#   getImage, then pixelToCameraPoint with the depth channel, then cameraToBase
#   cameraToBase uses a live TF lookup because the head pan and tilt change the camera pose
#
# readyPose and the tray poses are joint space poses

LEFT_ARM_JOINTS = ["Rotation_L", "Pitch_L", "Elbow_L", "Wrist_Pitch_L", "Wrist_Roll_L"]
RIGHT_ARM_JOINTS = ["Rotation_R", "Pitch_R", "Elbow_R", "Wrist_Pitch_R", "Wrist_Roll_R"]

# Jaw joint limits are the mechanical stops
GRIPPER_OPEN = 1.7453292
GRIPPER_CLOSED = -0.374533
# Closing fully would push the cube out, so the jaw stops on the cube
GRASP_JAW = 0.55
# Opening used during the approach
GRASP_OPEN = 1.0

# Fully extended zero pose
ARM_HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0]

# Joint poses that put the TCP over each arm spot in the tray, fingers pointing down
LEFT_TRAY_JOINTS = [-1.9181, 1.1113, 2.0946, 0.7083, -1.5708]
RIGHT_TRAY_JOINTS = [2.116, 1.1136, 2.099, 0.7062, -1.5708]

# Ready stance, both arms forward with the gripper down
READY_LEFT_JOINTS = [-1.600, 1.527, 2.123, 0.063, 1.664]
READY_RIGHT_JOINTS = [1.630, 1.525, 2.110, 0.074, 1.459]


def custom_thread_excepthook(args):
    if "spin" in args.thread.name:
        return
    sys.__excepthook__(args.exc_type, args.exc_value, args.exc_traceback)


threading.excepthook = custom_thread_excepthook

print("HAL (XLeRobot Home) initializing", flush=True)
if not rclpy.ok():
    rclpy.init(args=None)

### HAL INIT: base ###
motor_node = MotorsNode("/logistic_robot/cmd_vel", 4, 0.3)
odometry_node = OdometryNode("/logistic_robot/odom")
sim_time_node = SimTimeNode()

### HAL INIT: cameras ###
head_camera_node = CameraNode("/logistic_robot/head_camera/image")
left_arm_camera_node = CameraNode("/logistic_robot/left_arm_camera/image_raw")
right_arm_camera_node = CameraNode("/logistic_robot/right_arm_camera/image_raw")


class JointStateNode(Node):
    """Latest position and velocity of every joint, by name."""

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

### HAL INIT: grippers, joint level arm control, grasping ###
arm_node = Node("hal_arm_node")

# Read by the gz_link_attacher plugins of the world
grasp_pub = arm_node.create_publisher(Bool, "/gripper_auto_attach", 10)
graspable_pub = arm_node.create_publisher(String, "/graspable_objects", 10)

# Own node for the TF buffer, sharing arm_node left it with stale transforms
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


### HAL INIT: arm poses, one subscriber per side ###
class ArmPose:
    """Gripper pose, position in meters, orientation as both quaternion and
    roll/pitch/yaw in radians, same shape as odometry's Pose3d."""

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


left_pose_node = ArmPoseNode("hal_left_pose_node", "/logistic_robot/left_robpose/Robpose")
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
    msg.data = "red_box,yellow_box,blue_box"
    graspable_pub.publish(msg)


arm_node.create_timer(1.0, __publish_graspable_objects)


### HAL INIT: Robmove clients, one per arm ###
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
    """roll, pitch, yaw in radians, returns (qx, qy, qz, qw)."""
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
    """Base pose (x, y, z, yaw...), from odometry."""
    return odometry_node.getPose3d()


def getJointPositions():
    """{joint name: position in rad} for every arm, gripper, head and wheel
    joint, as last reported on /joint_states."""
    return dict(joint_state_node.positions)


def getSimTime():
    return sim_time_node.getSimTime()


def __sim_now():
    t = sim_time_node.getSimTime()
    return t.seconds + t.nanoseconds * 1e-9


def sleepSim(seconds):
    """Wait `seconds` of simulation time, the sim runs well below real time."""
    start = __sim_now()
    deadline = time.time() + seconds * 20 + 5
    while __sim_now() - start < seconds and time.time() < deadline:
        time.sleep(0.02)


def getImage():
    """Head camera image, BGR numpy array."""
    image = head_camera_node.getImage()
    while image is None:
        image = head_camera_node.getImage()
    return image.data


def getLeftArmImage():
    """Left wrist camera image, BGR numpy array."""
    image = left_arm_camera_node.getImage()
    while image is None:
        image = left_arm_camera_node.getImage()
    return image.data


def getRightArmImage():
    """Right wrist camera image, BGR numpy array."""
    image = right_arm_camera_node.getImage()
    while image is None:
        image = right_arm_camera_node.getImage()
    return image.data


# Cameras cannot be disabled in Gazebo, so off means a very low render rate
CAMERA_SET_RATE_SERVICES = {
    "head": "/logistic_robot/head_camera/set_rate",
    "left_arm": "/logistic_robot/left_arm_camera/image_raw/set_rate",
    "right_arm": "/logistic_robot/right_arm_camera/image_raw/set_rate",
}
CAMERA_ON_HZ = 15.0  # matches update_rate in xlerobot_gz.urdf.xacro
CAMERA_IDLE_HZ = 0.2


def setCameraRate(camera, hz):
    """Set a camera's render rate in Hz. camera is "head", "left_arm" or
    "right_arm". Returns True if Gazebo accepted it."""
    service = CAMERA_SET_RATE_SERVICES[camera]
    try:
        # gz service returns 0 even when the service does not exist
        info = subprocess.run(
            ["gz", "service", "-i", "-s", service], capture_output=True, text=True, timeout=10
        )
        if "Service providers" not in info.stdout:
            print(f"[HAL] setCameraRate: no Gazebo service {service}", flush=True)
            return False
        subprocess.run(
            ["gz", "service", "-s", service, "--reqtype", "gz.msgs.Double",
             "--reptype", "gz.msgs.Empty", "--timeout", "3000", "--req", f"data: {float(hz)}"],
            capture_output=True, text=True, timeout=10, check=True,
        )
    except (OSError, subprocess.SubprocessError) as e:
        print(f"[HAL] setCameraRate({camera}, {hz}) failed: {e}", flush=True)
        return False
    return True


def cameraOff(*cameras):
    """Slow the given cameras, all three when none are named."""
    return all(setCameraRate(c, CAMERA_IDLE_HZ) for c in (cameras or CAMERA_SET_RATE_SERVICES))


def cameraOn(*cameras):
    """Restore the render rate, the first fresh frame can take a few seconds."""
    return all(setCameraRate(c, CAMERA_ON_HZ) for c in (cameras or CAMERA_SET_RATE_SERVICES))


def getDepthImage():
    """Head camera depth, float32 meters, same pixel grid as getImage()."""
    while head_depth_node.depth is None:
        pass
    return head_depth_node.depth


def pixelToCameraPoint(u, v):
    """Back-project a pixel to a 3D point (x, y, z) in the head camera optical frame.
    Returns None if the pixel has no valid depth.
    """
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
    """Transform a point from the head camera optical frame to base_footprint.
    Returns None if TF or the depth frame is not available yet.
    """
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


def setV(velocity):
    """Forward speed in m/s."""
    motor_node.sendV(velocity)


def setVY(velocity):
    """Sideways speed in m/s."""
    motor_node.last_twist.linear.y = float(velocity)
    motor_node.pub.publish(motor_node.last_twist)


def setW(velocity):
    """Yaw rate in rad/s."""
    motor_node.sendW(velocity)


def moveLeftArm(x, y, z, roll=0.0, pitch=0.0, yaw=0.0, speed=0.3, motion="PTP"):
    """Move the left gripper to a Cartesian pose relative to base_footprint.

    motion is "PTP" for a coarse reposition or "LIN" for a straight line.
    Blocks until the move ends and returns True on success.
    """
    return __move_cartesian(LEFT_ARM, x, y, z, roll, pitch, yaw, speed, motion, "left")


def moveRightArm(x, y, z, roll=0.0, pitch=0.0, yaw=0.0, speed=0.3, motion="PTP"):
    """Move the right gripper to a Cartesian pose, same convention as moveLeftArm."""
    return __move_cartesian(
        RIGHT_ARM, x, y, z, roll, pitch, yaw, speed, motion, "right"
    )


def getLeftArmPose():
    """Current left gripper pose, or None if no Robpose message has arrived yet."""
    return left_pose_node.pose


def getRightArmPose():
    """Current right gripper pose, or None if no Robpose message has arrived yet."""
    return right_pose_node.pose


def setLeftArmJoints(positions, duration=2.0):
    """Raw joint move, no planning, no collision checking.

    positions is 5 joint values in radians, in this order
    Rotation_L, Pitch_L, Elbow_L, Wrist_Pitch_L, Wrist_Roll_L
    """
    __send_trajectory(left_arm_joint_client, LEFT_ARM_JOINTS, positions, duration)


def setRightArmJoints(positions, duration=2.0):
    """Raw joint move, same joint order as setLeftArmJoints."""
    __send_trajectory(right_arm_joint_client, RIGHT_ARM_JOINTS, positions, duration)


def setHeadJoints(pan, tilt, duration=1.0):
    """Raw head move, both joints together. Tilt goes from -0.76 up to 1.45 down."""
    __send_trajectory(head_joint_client, ["head_pan_joint", "head_tilt_joint"], [pan, tilt], duration)


def homeLeftArm(duration=2.0):
    """Send the left arm to its safe, fully extended zero pose."""
    setLeftArmJoints(ARM_HOME_JOINTS, duration)


def homeRightArm(duration=2.0):
    """Send the right arm to its safe, fully extended zero pose."""
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
    """Left gripper, True closes and arms auto attach, False opens and releases.
    target overrides the jaw angle.

    The auto attach switch is shared by both grippers, so grasp one arm at a time.
    """
    grasp_pub.publish(Bool(data=bool(closed)))
    if target is None:
        target = GRIPPER_CLOSED if closed else GRIPPER_OPEN
    __send_trajectory(left_gripper_client, ["Jaw_L"], [target], duration)


def setRightGripper(closed, duration=1.0, target=None):
    """Right gripper, same as setLeftGripper."""
    grasp_pub.publish(Bool(data=bool(closed)))
    if target is None:
        target = GRIPPER_CLOSED if closed else GRIPPER_OPEN
    __send_trajectory(right_gripper_client, ["Jaw_R"], [target], duration)


# Tray drop and pick, the cube stays at a fixed spot so a later close meets it again
def dropInLeftTray(speed=0.3):
    """Stow the held cube in the left arm's tray spot."""
    return __tray_visit("left", moveLeftArm, setLeftGripper, setLeftArmJoints, True, speed)


def dropInRightTray(speed=0.3):
    """Stow the held cube in the right arm's tray spot."""
    return __tray_visit("right", moveRightArm, setRightGripper, setRightArmJoints, True, speed)


def pickFromLeftTray(speed=0.3):
    """Take a cube from the left arm's tray spot (put there by dropInLeftTray)."""
    return __tray_visit("left", moveLeftArm, setLeftGripper, setLeftArmJoints, False, speed)


def pickFromRightTray(speed=0.3):
    """Take a cube from the right arm's tray spot."""
    return __tray_visit("right", moveRightArm, setRightGripper, setRightArmJoints, False, speed)


# Grasp geometry
# The arms have 5 joints so MoveIt only accepts exactly reachable poses
# The orientation is solved numerically and the forward kinematics pose is sent to Robmove
GRASP_PITCH = 0.4
GRASP_APPROACH_DISTANCE = 0.10

# Arm kinematic chain from the xacro, per joint origin xyz, origin rpy and axis
_ARM_CHAIN = [
    ((0, -0.0452, 0.0165), (1.5708, 0, 0), (0, -1, 0)),
    ((0, 0.1025, 0.0306), (1.5708, 0, 0), (-1, 0, 0)),
    ((0, 0.11257, 0.028), (-1.5708, 0, 0), (1, 0, 0)),
    ((0, 0.0052, 0.1349), (-1.5708, 0, 0), (1, 0, 0)),
    ((0, -0.0601, 0), (0, 1.5707927, 0), (0, -1, 0)),
]
_TCP_OFFSET = (-0.0171, -0.0800, 0.0)
_ARM_LOWER = np.array([-2.16, -0.22, -0.22, -1.6580628, -2.7438473])
_ARM_UPPER = np.array([2.16, 3.37, 3.14, 1.6580627, 2.8412063])
# Arm mounts in base_footprint, position and yaw
_ARM_MOUNT = {
    "left": (np.array([-0.09, -0.11, 0.765]), 0.0),
    "right": (np.array([-0.09, 0.11, 0.765]), math.pi),
}
_ARM_SEEDS = {
    "left": [
        [-1.6, 1.5, 2.1, 0.06, 1.571], [-1.6, 1.5, 2.1, 0.06, -1.571],
        [-1.0, 1.2, 1.8, 0.5, 1.0], [-1.0, 1.2, 1.8, 0.5, -1.0], [0.0, 1.0, 1.5, 0.3, 0.0],
    ],
    "right": [
        [1.63, 1.52, 2.11, 0.07, 1.571], [1.63, 1.52, 2.11, 0.07, -1.571],
        [1.0, 1.2, 1.8, 0.5, 1.0], [1.0, 1.2, 1.8, 0.5, -1.0], [0.0, 1.0, 1.5, 0.3, 0.0],
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
    """TCP position and rotation matrix in base_footprint for 5 joint angles."""
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
    """Rotation matrix and fingertip direction for a grasp at `target`, or
    None if the arm cannot get there."""
    from scipy.optimize import least_squares

    target = np.array(target, dtype=float)
    pitch = GRASP_PITCH if pitch is None else pitch

    def residual(q):
        p, R = __arm_fk(arm, q)
        f = -R[:, 1]  # fingers point along the TCP -y axis
        horizontal = np.array([f[0], f[1], 0.0])
        horizontal /= np.linalg.norm(horizontal)
        sideways = np.array([-horizontal[1], horizontal[0], 0.0])
        return np.concatenate(
            [(p - target) * 10.0, [math.asin(-f[2]) - pitch, R[:, 2] @ sideways]]
        )

    # Two exact solutions exist with wrist roll of plus or minus pi/2, roll_sign picks one
    solutions = []
    for seed in _ARM_SEEDS[arm]:
        r = least_squares(
            residual, seed, bounds=(_ARM_LOWER, _ARM_UPPER), xtol=1e-12, ftol=1e-12, gtol=1e-12
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
    """Joint angles reaching `position` with rotation R exactly, or None."""
    from scipy.optimize import least_squares

    def residual(q):
        p, Rq = __arm_fk(arm, q)
        return np.concatenate([(p - position) * 10.0, (Rq - R).ravel()])

    r = least_squares(
        residual, seed, bounds=(_ARM_LOWER, _ARM_UPPER), xtol=1e-12, ftol=1e-12, gtol=1e-12
    )
    return r.x if np.linalg.norm(r.fun) < 1e-6 else None


def __grasp(move, set_gripper, arm, x, y, z, speed):
    solved = __solve_grasp_orientation(arm, (x, y, z), roll_sign=1.0 if arm == "left" else -1.0)
    if solved is None:
        print(f"[HAL] {arm} arm cannot reach a grasp at ({x:.3f}, {y:.3f}, {z:.3f})", flush=True)
        return False
    R, f, q_grasp = solved
    roll, pitch, yaw = __rpy_from_matrix(R)
    pre = (
        x - f[0] * GRASP_APPROACH_DISTANCE,
        y - f[1] * GRASP_APPROACH_DISTANCE,
        z - f[2] * GRASP_APPROACH_DISTANCE,
    )
    set_gripper(False, 1.0, GRASP_OPEN)
    sleepSim(1.0)
    if not move(pre[0], pre[1], pre[2], roll, pitch, yaw, speed, "PTP"):
        return False
    if not move(x, y, z, roll, pitch, yaw, speed, "LIN"):
        return False
    set_gripper(True, 1.0, GRASP_JAW)
    sleepSim(1.0)

    # Retreat with fallbacks because a LIN move failed once for one arm
    for motion in ("LIN", "LIN", "PTP"):
        if move(pre[0], pre[1], pre[2], roll, pitch, yaw, speed, motion):
            return True
        print(f"[HAL] {arm} retreat with {motion} failed", flush=True)
    q_pre = __solve_pose(arm, pre, R, q_grasp)
    if q_pre is None:
        return False
    print(f"[HAL] {arm} retreat: sending joints straight to the controller", flush=True)
    (setLeftArmJoints if arm == "left" else setRightArmJoints)(q_pre, 3.0)
    sleepSim(3.0)
    return True


def __rpy_from_matrix(R):
    pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    return math.atan2(R[2, 1], R[2, 2]), pitch, math.atan2(R[1, 0], R[0, 0])


# Tray drop and pick, the hand goes via a carry pose and a waypoint above the spot
# so the cube clears the front rim of the tray
# TRAY_PITCH keeps the fingers nearly straight down
# TRAY_APPROACH_Z is the highest waypoint the wrist can reach
TRAY_PITCH = 1.45
TRAY_APPROACH_Z = 0.84
TRAY_DROP_XYZ = {"left": (0.062, -0.082, 0.724), "right": (0.062, 0.083, 0.724)}
TRAY_CARRY_JOINTS = {
    "left": [-1.544, 1.356, 1.66, 0.612, -1.571],
    "right": [1.86, 1.332, 1.621, 0.142, -1.571],
}


def __tray_visit(arm, move, set_gripper, set_joints, release, speed):
    """Carry, hover over the tray spot, lower, release or close, and come back up."""
    x, y, z = TRAY_DROP_XYZ[arm]
    solved = __solve_grasp_orientation(arm, (x, y, z), TRAY_PITCH, roll_sign=-1.0)
    if solved is None:
        print(f"[HAL] {arm} arm cannot reach its tray spot", flush=True)
        return False
    R, _, q_drop = solved
    q_above = __solve_pose(arm, (x, y, TRAY_APPROACH_Z), R, q_drop)
    if q_above is None:
        return False
    roll, pitch, yaw = __rpy_from_matrix(R)
    p_carry, R_carry = __arm_fk(arm, TRAY_CARRY_JOINTS[arm])
    c_roll, c_pitch, c_yaw = __rpy_from_matrix(R_carry)

    def go(label, q, target, orientation, motion):
        if move(target[0], target[1], target[2], *orientation, speed, motion):
            return
        print(f"[HAL] {arm} {label}: planning failed, sending joints straight to the controller", flush=True)
        set_joints(q, 3.0)
        sleepSim(3.0)

    if not release:
        set_gripper(False, 1.0, GRASP_OPEN)
        sleepSim(1.0)
    go("carry", TRAY_CARRY_JOINTS[arm], p_carry, (c_roll, c_pitch, c_yaw), "PTP")
    go("above the tray", q_above, (x, y, TRAY_APPROACH_Z), (roll, pitch, yaw), "PTP")
    go("lowering", q_drop, (x, y, z), (roll, pitch, yaw), "LIN")
    if release:
        set_gripper(False, 1.0, GRASP_OPEN)
    else:
        set_gripper(True, 1.0, GRASP_JAW)
    sleepSim(1.0)
    go("coming back up", q_above, (x, y, TRAY_APPROACH_Z), (roll, pitch, yaw), "LIN")
    return True


def graspLeft(x, y, z, speed=0.3):
    """Pick up what is at (x, y, z) in base_footprint with the left arm.
    Returns True only if every step succeeded.
    """
    return __grasp(moveLeftArm, setLeftGripper, "left", x, y, z, speed)


def graspRight(x, y, z, speed=0.3):
    """Same as graspLeft, right arm."""
    return __grasp(moveRightArm, setRightGripper, "right", x, y, z, speed)
