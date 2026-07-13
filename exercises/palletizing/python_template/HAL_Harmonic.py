print("HAL Harmonic initializing", flush=True)

import sys, os, time, math
import rclpy
import numpy as np
from sensor_msgs.msg import JointState

from rclpy.node import Node
from ros2srrc_data.msg import Robpose
from ament_index_python.packages import get_package_share_directory

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

# Objects the suction gripper is allowed to pick. The gz_link_attacher matches
# these as substrings of the colliding model name, so "box" covers every box
# the feeder spawns (box_<run_id>_<counter>).
GRASPABLE_OBJECTS = "box"

graspable_msg = String()
graspable_msg.data = GRASPABLE_OBJECTS
HAL.graspable_pub.publish(graspable_msg)

print("[HAL] Published graspable objects")


def publish_graspable_objects():

    graspable_msg = String()
    graspable_msg.data = GRASPABLE_OBJECTS
    HAL.graspable_pub.publish(graspable_msg)


# Periodic republish so late-joining subscribers receive the list
HAL.create_timer(1.0, publish_graspable_objects)

# ==============================================================
# CONVEYOR HANDSHAKE (feeder coordination)
# ==============================================================
# The conveyor feeder (box_spawner) and the robot coordinate with a two-message
# handshake so boxes are picked one at a time and never pile up:
#   - the feeder publishes a box name on /box_ready when a box is stopped at the
#     pickup point, then waits;
#   - the robot calls BoxDone(name) once the box is palletized, which publishes
#     on /box_done and releases the next box.
# Keeping this on the HAL node means the student's solution only ever calls
# WaitForBox()/BoxDone() and never touches ROS topics directly.

HAL._ready_box = None
HAL._processed_boxes = set()


def _on_box_ready(msg):
    # Ignore re-announcements of a box we've already handed to the solution.
    if msg.data not in HAL._processed_boxes:
        HAL._ready_box = msg.data


HAL.create_subscription(String, "/box_ready", _on_box_ready, 10)
HAL.box_done_pub = HAL.create_publisher(String, "/box_done", 10)


def WaitForBox():
    """Block until the feeder announces a box at the pickup point.

    Returns the box name (str). Use it as the handle to pass to BoxDone() once
    the box has been stacked on the pallet.
    """
    HAL._ready_box = None
    while rclpy.ok() and HAL._ready_box is None:
        rclpy.spin_once(HAL, timeout_sec=0.1)
    HAL._processed_boxes.add(HAL._ready_box)
    return HAL._ready_box


def BoxDone(name):
    """Tell the feeder the box has been palletized; releases the next box.

    Pass the name returned by WaitForBox().
    """
    HAL.box_done_pub.publish(String(data=name))
    print(f"[HAL] BoxDone({name}) -> released next box")


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


def _wait_motion_complete(timeout=15.0, vel_threshold=0.01, stable_count=8):
    """Block until all joint velocities stay below threshold for stable_count cycles."""
    latest = [None]

    sub = HAL.create_subscription(
        JointState, '/joint_states',
        lambda msg: latest.__setitem__(0, msg),
        10
    )
    stable = 0
    deadline = time.time() + timeout
    try:
        while time.time() < deadline:
            rclpy.spin_once(HAL, timeout_sec=0.05)
            msg = latest[0]
            if msg is not None and len(msg.velocity) > 0:
                if all(abs(v) < vel_threshold for v in msg.velocity):
                    stable += 1
                    if stable >= stable_count:
                        return True
                else:
                    stable = 0
    finally:
        HAL.destroy_subscription(sub)
    return False


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
        _wait_motion_complete()   # block until joint velocities settle
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

    auto_msg = Bool()
    auto_msg.data = bool(on)
    HAL.auto_attach_pub.publish(auto_msg)

    print(f"[HAL] Suction {'ENABLED' if on else 'DISABLED'}")

    time.sleep(wait_time)

    print(f"[HAL] Waiting {wait_time} s")
    print("==================================================\n")
