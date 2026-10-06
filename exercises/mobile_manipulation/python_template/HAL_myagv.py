import os
import sys
import threading
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool, Empty, Int32, String
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import (
    OdometryNode,
    quat2Yaw,
    quat2Pitch,
    quat2Roll,
)
from hal_interfaces.general.laser import LaserNode
from hal_interfaces.general.camera import CameraNode
from hal_interfaces.general.sim_time import SimTimeNode

# RBT client of ros2srrc_execution as in pick_place
sys.path.append(
    os.path.join(get_package_share_directory("ros2srrc_execution"), "python", "robot")
)
from robot import RBT  # noqa: E402
from ros2srrc_data.msg import Robpose  # noqa: E402

# HAL for the myAGV with a mechArm 270 in the warehouse exercise
# moveArm goals are planned with MoveIt2 and are relative to base_footprint
# so they stay valid while the robot drives
# worldToBase brings a warehouse point into that frame

ARM_JOINTS = [
    "joint1_to_base",
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
]

# Joint that drives the adaptive gripper and its open and closed angles
GRIPPER_JOINT = "gripper_base_to_gripper_left3"
GRIPPER_OPEN = 0.15
# Closes to about 22 mm so it squeezes the objects without sinking into them
GRIPPER_CLOSED = -0.4

# Arm folded over the base for driving
ARM_HOME_JOINTS = [0.0, -0.1, 0.2, 0.0, 1.5, 0.0]

# Gripper pitched down ahead of the base where floor picks start
ARM_READY_JOINTS = [0.0, 0.236, 0.255, 0.0, 0.739, 0.0]

# Gripper 70 degrees below the horizon with the fingers closing sideways
# The wrist of the mechArm cannot point straight down this close to the base
GRASP_ROLL = -math.pi + math.radians(20)
GRASP_YAW = -math.pi / 2

GRASPABLE_OBJECTS = "red_ball,blue_ball"


def custom_thread_excepthook(args):
    if "spin" in args.thread.name:
        return
    sys.__excepthook__(args.exc_type, args.exc_value, args.exc_traceback)


threading.excepthook = custom_thread_excepthook

print("HAL (myAGV mechArm Warehouse) initializing", flush=True)
if not rclpy.ok():
    rclpy.init(args=None)

### HAL INIT ###
motor_node = MotorsNode("/myagv_mecharm/cmd_vel", 4, 0.3)
odometry_node = OdometryNode("/myagv_mecharm/odom")
laser_node = LaserNode("/myagv_mecharm/scan")
camera_node = CameraNode("/myagv_mecharm/camera/image_raw")
sim_time_node = SimTimeNode()


class JointStateNode(Node):
    """Latest position of every joint by name."""

    def __init__(self, topic):
        super().__init__("hal_joint_state_node")
        self.positions = {}
        self.create_subscription(JointState, topic, self.__callback, 10)

    def __callback(self, msg):
        for i, name in enumerate(msg.name):
            self.positions[name] = msg.position[i]


joint_state_node = JointStateNode("/myagv_mecharm/joint_states")


class DeliveryNode(Node):
    """Ball requests and target stations of the warehouse delivery world."""

    def __init__(self):
        super().__init__("hal_delivery_node")
        self.targets = {"red": 0, "blue": 0}
        self.spawn_pub = {
            c: self.create_publisher(Empty, f"/warehouse_delivery/spawn_{c}_ball", 10)
            for c in ("red", "blue")
        }
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for c in ("red", "blue"):
            self.create_subscription(
                Int32,
                f"/warehouse_delivery/{c}_target",
                lambda msg, c=c: self.targets.__setitem__(c, msg.data),
                latched,
            )


delivery_node = DeliveryNode()

# Gripper and joint level arm control
arm_node = Node("hal_arm_node")

# Read by the gz_link_attacher plugin of the world
grasp_pub = arm_node.create_publisher(Bool, "/gripper_auto_attach", 10)
graspable_pub = arm_node.create_publisher(String, "/graspable_objects", 10)


def __joint_client(controller_name):
    client = ActionClient(
        arm_node,
        FollowJointTrajectory,
        f"/myagv_mecharm/{controller_name}/follow_joint_trajectory",
    )
    print(f"[HAL] waiting for {controller_name}...", flush=True)
    while not client.wait_for_server(timeout_sec=1.0):
        print(f"[HAL] still waiting for {controller_name}...", flush=True)
    print(f"[HAL] {controller_name} ready", flush=True)
    return client


arm_joint_client = __joint_client("arm_controller")
gripper_client = __joint_client("gripper_controller")


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


# Gripper pose
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
    def __init__(self, topic):
        super().__init__("hal_arm_pose_node")
        self.pose = None
        self.create_subscription(Robpose, topic, self.__callback, 10)

    def __callback(self, msg):
        self.pose = ArmPose(msg)


arm_pose_node = ArmPoseNode("/myagv_mecharm/Robpose")

executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(odometry_node)
executor.add_node(laser_node)
executor.add_node(camera_node)
executor.add_node(sim_time_node)
executor.add_node(joint_state_node)
executor.add_node(arm_node)
executor.add_node(arm_pose_node)
executor.add_node(delivery_node)


def __auto_spin() -> None:
    try:
        executor.spin()
    except Exception:
        pass


executor_thread = threading.Thread(target=__auto_spin, daemon=True)
executor_thread.start()


def __publish_graspable_objects():
    graspable_pub.publish(String(data=GRASPABLE_OBJECTS))


arm_node.create_timer(1.0, __publish_graspable_objects)

# Robmove client
print("[HAL] connecting to Robmove action...", flush=True)
ARM = RBT(robmove_action="/myagv_mecharm/Robmove", use_move=False)


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


def getPose3d():
    """Base pose from odometry."""
    return odometry_node.getPose3d()


def getLaserData():
    """360 degree laser scan."""
    return laser_node.getLaserData()


def getImage():
    """Front camera image in BGR."""
    image = camera_node.getImage()
    while image is None:
        image = camera_node.getImage()
    return image.data


def getJointPositions():
    """Last reported position in radians of every joint by name."""
    return dict(joint_state_node.positions)


def spawnRedBall():
    """Drop a red ball in the red dispenser and light its target station."""
    delivery_node.spawn_pub["red"].publish(Empty())


def spawnBlueBall():
    """Drop a blue ball in the blue dispenser and light its target station."""
    delivery_node.spawn_pub["blue"].publish(Empty())


def getRedTarget():
    """Station number where the red ball goes or 0 when there is none."""
    return delivery_node.targets["red"]


def getBlueTarget():
    """Station number where the blue ball goes or 0 when there is none."""
    return delivery_node.targets["blue"]


def getSimTime():
    return sim_time_node.getSimTime()


def __sim_now():
    t = sim_time_node.getSimTime()
    return t.seconds + t.nanoseconds * 1e-9


def sleepSim(seconds):
    """Wait in simulation time because the simulation can run slower than real time."""
    start = __sim_now()
    deadline = time.time() + seconds * 20 + 5
    while __sim_now() - start < seconds and time.time() < deadline:
        time.sleep(0.02)


def worldToBase(x, y, z):
    """Warehouse point expressed in base_footprint using odometry."""
    pose = odometry_node.getPose3d()
    dx = x - pose.x
    dy = y - pose.y
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return (c * dx + s * dy, -s * dx + c * dy, z)


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


def __robmove(x, y, z, roll, pitch, yaw, speed, motion):
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
        f"[HAL] arm {motion} to xyz=({x:.3f}, {y:.3f}, {z:.3f}) "
        f"rpy=({roll:.2f}, {pitch:.2f}, {yaw:.2f})",
        flush=True,
    )

    result = ARM.RobMove_EXECUTE(motion, float(speed), target)

    if result["Success"]:
        print(f"[HAL] arm move done in {result['ExecTime']} s", flush=True)
    else:
        print(f"[HAL] arm move FAILED: {result['Message']}", flush=True)

    return result["Success"]


def moveArm(
    x, y, z, roll=GRASP_ROLL, pitch=0.0, yaw=GRASP_YAW, speed=0.3, motion="PTP"
):
    """Move the gripper center to a pose relative to base_footprint.
    The default orientation looks forward 70 degrees below the horizon.
    The arm reaches the floor about 0.25 m ahead of the base.
    motion is PTP for a free move or LIN for a straight line.
    A failed PTP is retried from the ready pose.
    Blocks until the move ends and returns True on success.
    """
    if __robmove(x, y, z, roll, pitch, yaw, speed, motion):
        return True

    # Pilz sometimes returns a trajectory the controller rejects so the move is tried again
    if __robmove(x, y, z, roll, pitch, yaw, speed, motion):
        return True

    if motion == "LIN":
        print("[HAL] falling back to PTP", flush=True)
        return __robmove(x, y, z, roll, pitch, yaw, speed, "PTP")

    print("[HAL] retrying from the ready pose", flush=True)
    readyArm()
    sleepSim(4.0)
    if __robmove(x, y, z, roll, pitch, yaw, speed, motion):
        return True
    # The fingers are symmetric so half a turn of the gripper grasps the same way
    return __robmove(x, y, z, roll, pitch, yaw + math.pi, speed, motion)


def getArmPose():
    """Current gripper pose or None until the first Robpose message."""
    return arm_pose_node.pose


def setArmJoints(positions, duration=3.0):
    """Move the arm joints directly without planning or collision checks.
    positions follows the order of ARM_JOINTS in radians.
    """
    __send_trajectory(arm_joint_client, ARM_JOINTS, positions, duration)


def homeArm(duration=3.0):
    """Fold the arm over the base before driving."""
    setArmJoints(ARM_HOME_JOINTS, duration)


def readyArm(duration=3.0):
    """Hold the gripper ahead of the base pitched down.
    Go here before reaching for the floor so the arm does not sweep into it.
    """
    setArmJoints(ARM_READY_JOINTS, duration)


def setGripper(closed, duration=1.0, target=None):
    """True closes the gripper and arms auto attach and False opens it.
    target overrides the gripper angle between GRIPPER_OPEN and GRIPPER_CLOSED.
    """
    grasp_pub.publish(Bool(data=bool(closed)))
    if target is None:
        target = GRIPPER_CLOSED if closed else GRIPPER_OPEN
    __send_trajectory(gripper_client, [GRIPPER_JOINT], [target], duration)
