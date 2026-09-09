print("HAL Harmonic initializing", flush=True)

import sys, os, time, math
import xml.etree.ElementTree as ET
import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from ros2srrc_data.msg import Robpose
from linkattacher_msgs.srv import AttachLink, DetachLink
from rcl_interfaces.srv import GetParameters
from ament_index_python.packages import get_package_share_directory

# Gripper (NO modificar según tu requisito)
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from std_msgs.msg import Bool, String

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

print("[HAL] Waiting for gripper controller...")
while not HAL.gripper_client.wait_for_server(timeout_sec=1.0):
    print("[HAL] Waiting for gripper controller...")
print("[HAL] Gripper ready")

print("[HAL] LinkAttacher ready")


def get_gripper_joint_name():
    """Read the gripper_controller joint list so this HAL works with any
    arm's gripper, not just the Robotiq one."""

    client = HAL.create_client(GetParameters, "/gripper_controller/get_parameters")

    if not client.wait_for_service(timeout_sec=5.0):
        return "robotiq_85_left_knuckle_joint"

    request = GetParameters.Request()
    request.names = ["joints"]

    future = client.call_async(request)
    rclpy.spin_until_future_complete(HAL, future)
    result = future.result()

    if result and result.values and result.values[0].string_array_value:
        return result.values[0].string_array_value[0]

    return "robotiq_85_left_knuckle_joint"


def get_gripper_joint_limits(joint_name):
    """Read the joint's <limit> from robot_description instead of assuming
    the Robotiq range. Every gripper has its own travel."""

    qos = QoSProfile(depth=1)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = QoSReliabilityPolicy.RELIABLE

    received = {}

    def on_robot_description(msg):
        received["urdf"] = msg.data

    subscription = HAL.create_subscription(
        String, "/robot_description", on_robot_description, qos
    )

    deadline = time.time() + 5.0
    while "urdf" not in received and time.time() < deadline:
        rclpy.spin_once(HAL, timeout_sec=0.2)

    HAL.destroy_subscription(subscription)

    if "urdf" not in received:
        return 0.0, 0.80285

    root = ET.fromstring(received["urdf"])

    for joint in root.findall("joint"):
        if joint.get("name") == joint_name:
            limit = joint.find("limit")
            if limit is not None:
                return float(limit.get("lower", 0.0)), float(limit.get("upper", 1.0))

    return 0.0, 0.80285


GRIPPER_JOINT_NAME = get_gripper_joint_name()
GRIPPER_MIN, GRIPPER_MAX = get_gripper_joint_limits(GRIPPER_JOINT_NAME)

print(f"[HAL] Gripper joint: {GRIPPER_JOINT_NAME} range [{GRIPPER_MIN}, {GRIPPER_MAX}]")

# blue_ball, green_cylinder, yellow_box, red_box are the UR5 world's
# objects, red_cube, green_cube, blue_cube are the Dobot world's
graspable_msg = String()

graspable_msg.data = "blue_ball,green_cylinder,yellow_box,red_box,red_cube,green_cube,blue_cube"

HAL.graspable_pub.publish(graspable_msg)

print("[HAL] Published graspable objects")


def publish_graspable_objects():

    graspable_msg = String()

    graspable_msg.data = (
        "blue_ball,green_cylinder,yellow_box,red_cube,green_cube,blue_cube"
    )

    HAL.graspable_pub.publish(graspable_msg)


HAL.create_timer(1.0, publish_graspable_objects)

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

    position = GRIPPER_MIN + ((GRIPPER_MAX - GRIPPER_MIN) * (relative_closure / 100.0))

    print(f"[HAL] Target gripper joint position: {position}")

    goal_msg = FollowJointTrajectory.Goal()

    goal_msg.trajectory.joint_names = [GRIPPER_JOINT_NAME]

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
