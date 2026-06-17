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

graspable_msg = String()

graspable_msg.data = "blue_ball,green_cylinder,yellow_box,red_box"

HAL.graspable_pub.publish(graspable_msg)

print("[HAL] Published graspable objects")


def publish_graspable_objects():

    graspable_msg = String()

    graspable_msg.data = "blue_ball," "green_cylinder," "yellow_box"

    HAL.graspable_pub.publish(graspable_msg)


HAL.create_timer(1.0, publish_graspable_objects)

# ==============================================================
# MoveAbsJ (IDÉNTICO a classic)
# ==============================================================


def MoveAbsJ(absolute_joints, speed, wait_time):

    ACTION = Action()
    ACTION.action = "MoveJ"
    ACTION.speed = float(speed)

    INPUT = Joints()
    INPUT.joint1 = float(absolute_joints[0])
    INPUT.joint2 = float(absolute_joints[1])
    INPUT.joint3 = float(absolute_joints[2])
    INPUT.joint4 = float(absolute_joints[3])
    INPUT.joint5 = float(absolute_joints[4])
    INPUT.joint6 = float(absolute_joints[5])
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

    # If closing -> enable contact detection
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

    max_open = 1.0
    min_close = 0.0

    position = min_close + ((max_open - min_close) * (relative_closure / 100.0))

    print(f"[HAL] Target gripper joint position: {position}")

    goal_msg = FollowJointTrajectory.Goal()

    goal_msg.trajectory.joint_names = ["robotiq_85_left_knuckle_joint"]

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
