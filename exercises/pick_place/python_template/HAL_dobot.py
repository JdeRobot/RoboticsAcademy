print("HAL Harmonic initializing", flush=True)

import sys, os, time, math
import rclpy
import numpy as np

from rclpy.node import Node
from ros2srrc_data.msg import Robpose
from linkattacher_msgs.srv import AttachLink, DetachLink
from ament_index_python.packages import get_package_share_directory

# Gripper (NO modificar según tu requisito)
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from std_msgs.msg import Bool, Float64MultiArray, String

# Paths
PATH = os.path.join(get_package_share_directory("ros2srrc_execution"), "python")

sys.path.append(PATH + "/robot")
from robot import RBT

# ROS msgs
from ros2srrc_data.msg import Action, Joint, Joints, Xyz, Ypr

# Init
rclpy.init(args=None)
UR5 = RBT()

HAL = Node("hal_node")
HAL.grasped_object = None

HAL.auto_attach_pub = HAL.create_publisher(
    Bool,
    "/gripper_auto_attach",
    10,
)

HAL.graspable_pub = HAL.create_publisher(
    String,
    "/graspable_objects",
    10,
)

HAL.gripper_client = ActionClient(
    HAL, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory"
)
HAL.gripper_ready = False

# The jaws rest closed and open towards negative positions
GRIPPER_JOINT = "joint_prismatic_l"
GRIPPER_OPEN = -0.018
GRIPPER_CLOSED = 0.0

# Latest arm state from the robpose node, read by get_Joint_states and get_TCP_pose
HAL.joints = None
HAL.pose = None


def on_joints(msg):
    HAL.joints = list(msg.data)


def on_pose(msg):
    HAL.pose = msg


HAL.create_subscription(Float64MultiArray, "/Robjoints", on_joints, 1)
HAL.create_subscription(Robpose, "/Robpose", on_pose, 1)

print("[HAL] LinkAttacher ready")


def wait_gripper():
    """Wait for the gripper controller on first use so the suction cup arm never blocks on it."""

    if not HAL.gripper_ready:
        print("[HAL] Waiting for gripper controller...")
        while not HAL.gripper_client.wait_for_server(timeout_sec=1.0):
            print("[HAL] Waiting for gripper controller...")
        print("[HAL] Gripper ready")
        HAL.gripper_ready = True


graspable_msg = String()

graspable_msg.data = "red_cube,green_cube,blue_cube"

HAL.graspable_pub.publish(graspable_msg)

print("[HAL] Published graspable objects")


def publish_graspable_objects():

    graspable_msg = String()

    graspable_msg.data = "red_cube,green_cube,blue_cube"

    HAL.graspable_pub.publish(graspable_msg)


HAL.create_timer(1.0, publish_graspable_objects)

# ==============================================================
# ROBOT INFO, same names as machine_vision
# ==============================================================


def wait_fresh(attribute):
    """Nothing spins HAL between calls, so wait for a message newer than the call."""

    setattr(HAL, attribute, None)
    deadline = time.time() + 2.0
    while getattr(HAL, attribute) is None and time.time() < deadline:
        rclpy.spin_once(HAL, timeout_sec=0.1)
    return getattr(HAL, attribute)


def get_Joint_states():
    """Current joint angles in degrees, in the order MoveAbsJ takes them."""

    joints = wait_fresh("joints")
    if joints is None:
        print("[HAL] ERROR: no joint state received")
        return None
    joints = [round(value, 3) for value in joints]
    print(f"Current Joint States (degrees): {joints}")
    return joints


def get_TCP_pose():
    """Current tool pose as (xyz, ypr), in the same frame, units and angle order
    MoveLinear takes, so it can be passed back to it."""

    pose = wait_fresh("pose")
    if pose is None:
        print("[HAL] ERROR: no tool pose received")
        return None

    qx, qy, qz, qw = pose.qx, pose.qy, pose.qz, pose.qw

    # Inverse of the quaternion built in MoveLinear
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

    xyz = [round(pose.x, 4), round(pose.y, 4), round(pose.z, 4)]
    ypr = [round(math.degrees(angle), 2) for angle in (roll, pitch, yaw)]
    print(f"Current TCP Pose - XYZ: {xyz}, YPR: {ypr}")
    return xyz, ypr


# ==============================================================
# MoveAbsJ (IDÉNTICO a classic)
# ==============================================================


def MoveAbsJ(absolute_joints, speed, wait_time):

    ACTION = Action()
    ACTION.action = "MoveJ"
    ACTION.speed = float(speed)

    # Only as many joints as this arm actually has get sent, movej.cpp
    # reads just the first N fields for an N-DOF group and ignores the rest.
    INPUT = Joints()
    joint_fields = [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "joint7",
    ]
    for field, value in zip(joint_fields, absolute_joints):
        setattr(INPUT, field, float(value))
    ACTION.movej = INPUT

    EXECUTION = UR5.Move_EXECUTE(ACTION)

    if EXECUTION["Success"]:
        print(f"Robot moved to Joint Angular Goal: {absolute_joints}")
        print(
            f"Movement Execution Time: {EXECUTION['ExecTime']} s at Robot Speed: {speed*100} %"
        )
    else:
        print("Robot movement FAILED, check REASON in MoveIt output")

    time.sleep(wait_time)
    print(f"Waiting {wait_time} s\n")


# ==============================================================
# MoveLinear
# ==============================================================


def MoveLinear(abs_xyz, abs_ypr, speed, wait_time):

    roll = math.radians(abs_ypr[0])  # Converts XYR to rad
    pitch = math.radians(abs_ypr[1])
    yaw = math.radians(abs_ypr[2])

    # Quaternion from YPT in rad
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)

    InputPose = Robpose()
    InputPose.x = float(abs_xyz[0])
    InputPose.y = float(abs_xyz[1])
    InputPose.z = float(abs_xyz[2])
    InputPose.qx = qx
    InputPose.qy = qy
    InputPose.qz = qz
    InputPose.qw = qw

    EXECUTION = UR5.RobMove_EXECUTE("LIN", float(speed), InputPose)

    # Print movement results if movement succeeded
    if EXECUTION["Success"] == True:
        print(f"Robot moved linearly to Abs XYZ: {abs_xyz} and Abs YPR: {abs_ypr}")
        print(
            f"Movement Execution Time: {EXECUTION['ExecTime']} s at Robot Speed: {speed*100} %"
        )
    else:
        print("Robot movement FAILED, check REASON in MoveIt output")

    # Wait till next movement
    time.sleep(wait_time)
    print(f"Waiting {wait_time} s")
    print("")


# ==============================================================
# MoveJoint
# ==============================================================


def MoveJoint(abs_xyz, abs_ypr, speed, wait_time):

    roll = math.radians(abs_ypr[0])
    pitch = math.radians(abs_ypr[1])
    yaw = math.radians(abs_ypr[2])

    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)

    InputPose = Robpose()
    InputPose.x = float(abs_xyz[0])
    InputPose.y = float(abs_xyz[1])
    InputPose.z = float(abs_xyz[2])
    InputPose.qx = qx
    InputPose.qy = qy
    InputPose.qz = qz
    InputPose.qw = qw

    EXECUTION = UR5.RobMove_EXECUTE("PTP", float(speed), InputPose)

    if EXECUTION["Success"]:
        print(
            f"Robot moved Point-to-Point to Abs XYZ: {abs_xyz} and Abs YPR: {abs_ypr}"
        )
        print(
            f"Movement Execution Time: {EXECUTION['ExecTime']} s at Robot Speed: {speed*100} %"
        )
    else:
        print("Robot movement FAILED, check REASON in MoveIt output")

    time.sleep(wait_time)
    print(f"Waiting {wait_time} s\n")


# ==============================================================
# MoveRelLinear
# ==============================================================


def MoveRelLinear(relative_xyz, speed, wait_time):

    ACTION = Action()
    ACTION.action = "MoveL"
    ACTION.speed = float(speed)

    INPUT = Xyz()
    INPUT.x = float(relative_xyz[0])
    INPUT.y = float(relative_xyz[1])
    INPUT.z = float(relative_xyz[2])
    ACTION.movel = INPUT

    EXECUTION = UR5.Move_EXECUTE(ACTION)

    if EXECUTION["Success"]:
        print(f"Robot moved LINEARLY by a relative increment of : {relative_xyz}")
        print(
            f"Movement Execution Time: {EXECUTION['ExecTime']} s at Robot Speed: {speed*100} %"
        )
    else:
        print("Robot movement FAILED, check REASON in MoveIt output")

    time.sleep(wait_time)
    print(f"Waiting {wait_time} s\n")


# ==============================================================
# MoveRelReor
# ==============================================================


def MoveRelReor(relative_ypr, speed, wait_time):

    ACTION = Action()
    ACTION.action = "MoveROT"
    ACTION.speed = float(speed)

    INPUT = Ypr()
    INPUT.pitch = float(relative_ypr[0])
    INPUT.yaw = float(relative_ypr[1])
    INPUT.roll = float(relative_ypr[2])
    ACTION.moverot = INPUT

    EXECUTION = UR5.Move_EXECUTE(ACTION)

    if EXECUTION["Success"]:
        print(f"TCP reoriented by a relative increment of : {relative_ypr}")
        print(
            f"Movement Execution Time: {EXECUTION['ExecTime']} s at Robot Speed: {speed*100} %"
        )
    else:
        print("Robot movement FAILED, check REASON in MoveIt output")

    time.sleep(wait_time)
    print(f"Waiting {wait_time} s\n")


# ==============================================================
# MoveSingleJ
# ==============================================================


def MoveSingleJ(joint_number, relative_angle, speed, wait_time):

    ACTION = Action()
    ACTION.action = "MoveR"
    ACTION.speed = float(speed)

    # Accepts 3 as well as "joint3"
    if isinstance(joint_number, int):
        joint_number = f"joint{joint_number}"

    INPUT = Joint()
    INPUT.joint = str(joint_number)
    INPUT.value = float(relative_angle)
    ACTION.mover = INPUT

    EXECUTION = UR5.Move_EXECUTE(ACTION)

    if EXECUTION["Success"]:
        print(f"Robot moved {joint_number} in {relative_angle} degrees")
        print(
            f"Movement Execution Time: {EXECUTION['ExecTime']} s at Robot Speed: {speed*100} %"
        )
    else:
        print("Robot movement FAILED, check REASON in MoveIt output")

    time.sleep(wait_time)
    print(f"Waiting {wait_time} s\n")


# ==============================================================
# GRIPPER CONTROL
# ==============================================================


def GripperSet(relative_closure, wait_time):
    """
    0%   = open
    100% = closed
    """

    print("\n==================================================")
    print("[HAL] GripperSet() called")
    print(f"[HAL] Requested closure: {relative_closure} %")
    print(f"[HAL] Wait time: {wait_time} s")

    wait_gripper()

    # ==========================================================
    # ENABLE/DISABLE AUTO ATTACH
    # ==========================================================

    auto_msg = Bool()

    # If closing, enable contact detection
    if relative_closure > 5:

        auto_msg.data = True

        print("[HAL] AutoAttach ENABLED")

    else:

        auto_msg.data = False

        print("[HAL] AutoAttach DISABLED")

    HAL.auto_attach_pub.publish(auto_msg)

    print("[HAL] AutoAttach message published")

    # ==========================================================
    # GRIPPER MOTION
    # ==========================================================

    position = GRIPPER_OPEN + (GRIPPER_CLOSED - GRIPPER_OPEN) * relative_closure / 100.0

    print(f"[HAL] Target gripper joint position: {position}")

    goal_msg = FollowJointTrajectory.Goal()

    goal_msg.trajectory.joint_names = [GRIPPER_JOINT]

    point = JointTrajectoryPoint()

    point.positions = [position]
    point.time_from_start = Duration(sec=1)

    goal_msg.trajectory.points.append(point)

    print("[HAL] Sending gripper trajectory...")

    future = HAL.gripper_client.send_goal_async(goal_msg)

    rclpy.spin_until_future_complete(HAL, future)

    goal_handle = future.result()

    if not goal_handle.accepted:

        print("[HAL] ERROR: Gripper trajectory rejected")
        return

    print("[HAL] Gripper trajectory accepted")

    result_future = goal_handle.get_result_async()

    rclpy.spin_until_future_complete(HAL, result_future)

    print("[HAL] Gripper motion completed")

    time.sleep(wait_time)

    print(f"[HAL] Waiting {wait_time} s")
    print("==================================================\n")


# ==============================================================
# SUCTION GRIPPER CONTROL
# ==============================================================


def SuctionSet(on, wait_time):
    """
    Turn the suction (vacuum) gripper on or off.

    on = True   -> grip: any graspable object touching the cup is attached
    on = False  -> release: the held object is detached

    The suction cup has no actuated joint and no controller. Gripping is handled
    by the gz_link_attacher plugin, which rigidly attaches an object to the cup
    link on contact while auto-attach is enabled and releases it when disabled.
    """

    print("\n==================================================")
    print(f"[HAL] SuctionSet({on}) called")

    # Nothing spins HAL here so the graspable list is sent again on every call
    publish_graspable_objects()

    auto_msg = Bool()
    auto_msg.data = bool(on)
    HAL.auto_attach_pub.publish(auto_msg)

    print(f"[HAL] Suction {'ENABLED' if on else 'DISABLED'}")

    time.sleep(wait_time)

    print(f"[HAL] Waiting {wait_time} s")
    print("==================================================\n")
