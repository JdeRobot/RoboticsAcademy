print("HAL Harmonic initializing", flush=True)

##############################################################################
# Robotics Academy – Pick and Place HAL (Gazebo Harmonic Version)
#
# This Hardware Abstraction Layer (HAL) provides:
#   - Robot kinematics control using MoveIt2 (IK service)
#   - Joint trajectory execution using ros2_control
#   - Relative Cartesian and orientation movements
#   - Gripper control using joint trajectory controller
#
# IMPORTANT:
# In Gazebo Harmonic, object grasping is handled by PHYSICS.
# No artificial attach plugin is used.
# The object is held by friction and contact forces.
#
# Architecture:
#   MoveJoint()  →  Compute IK  →  Send joint trajectory
#   GripperSet() →  Send gripper joint trajectory
#
##############################################################################

import rclpy
import math
import time
import numpy as np

from rclpy.node import Node
from rclpy.action import ActionClient

# MoveIt2 interfaces
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionIK

# ros2_control interfaces
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Geometry messages
from geometry_msgs.msg import PoseStamped


# ==============================================================
# MAIN ROS2 NODE
# ==============================================================

class HarmonicHAL(Node):
    """
    Main ROS2 node that interfaces with:
      - MoveIt2 (Inverse Kinematics)
      - Joint trajectory controller (arm)
      - Joint trajectory controller (gripper)

    This node acts as the bridge between high-level commands
    and low-level motion execution.
    """

    def __init__(self):
        super().__init__("harmonic_hal")

        # Action client for MoveIt motion planning (not directly used for execution)
        self.movegroup_client = ActionClient(self, MoveGroup, "/move_action")

        # Action client for arm joint control (ros2_control)
        self.joint_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )

        # Service client for inverse kinematics computation
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        print("Waiting for MoveGroup...")
        self.movegroup_client.wait_for_server()

        print("Waiting for joint controller...")
        self.joint_client.wait_for_server()

        print("Waiting for IK service...")
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            pass

        # Action client for gripper joint controller
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
        )

        print("Waiting for gripper controller...")
        self.gripper_client.wait_for_server()

        # Logical state of grasped object (for debugging only)
        self.grasped_object = None

        print("All systems ready")

        # Internal state tracking (last commanded pose)
        self.current_pose = [0.5, 0.0, 0.8]
        self.current_orientation = [0, 90, 0]


# ==============================================================
# NODE INITIALIZATION
# ==============================================================

rclpy.init()
ROBOT = HarmonicHAL()


# ==============================================================
# MoveAbsJ – Direct Joint Control
# ==============================================================

def MoveAbsJ(joints_deg, speed, wait_time):
    """
    Moves the robot to an absolute joint configuration.

    This function directly commands the joint trajectory controller
    without using inverse kinematics.
    """

    goal_msg = FollowJointTrajectory.Goal()

    goal_msg.trajectory.joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]

    # Convert degrees to radians
    point = JointTrajectoryPoint()
    point.positions = [math.radians(j) for j in joints_deg]
    point.time_from_start = Duration(sec=3)

    goal_msg.trajectory.points.append(point)

    future = ROBOT.joint_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(ROBOT, future)
    goal_handle = future.result()

    if not goal_handle.accepted:
        print("MoveAbsJ rejected")
        return

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(ROBOT, result_future)

    print("MoveAbsJ executed")
    time.sleep(wait_time)


# ==============================================================
# MoveJoint – Cartesian Movement via IK
# ==============================================================

def MoveJoint(abs_xyz, abs_ypr, speed, wait_time):
    """
    Moves robot to an absolute Cartesian pose.

    Steps:
      1. Convert YPR to quaternion
      2. Request IK solution from MoveIt2
      3. Send resulting joint values to trajectory controller
    """

    # Convert Euler angles (degrees) to quaternion
    roll = math.radians(abs_ypr[0])
    pitch = math.radians(abs_ypr[1])
    yaw = math.radians(abs_ypr[2])

    qx = np.sin(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) - np.cos(roll/2)*np.sin(pitch/2)*np.sin(yaw/2)
    qy = np.cos(roll/2)*np.sin(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)*np.cos(pitch/2)*np.sin(yaw/2)
    qz = np.cos(roll/2)*np.cos(pitch/2)*np.sin(yaw/2) - np.sin(roll/2)*np.sin(pitch/2)*np.cos(yaw/2)
    qw = np.cos(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)*np.sin(pitch/2)*np.sin(yaw/2)

    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.pose.position.x = float(abs_xyz[0])
    pose.pose.position.y = float(abs_xyz[1])
    pose.pose.position.z = float(abs_xyz[2])
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    # Request inverse kinematics solution
    request = GetPositionIK.Request()
    request.ik_request.group_name = "ur5_manipulator"
    request.ik_request.ik_link_name = "tool0"
    request.ik_request.pose_stamped = pose
    request.ik_request.timeout.sec = 2
    request.ik_request.avoid_collisions = True

    future = ROBOT.ik_client.call_async(request)

    timeout = 3.0
    start_time = time.time()

    while not future.done():
        rclpy.spin_once(ROBOT, timeout_sec=0.1)
        if time.time() - start_time > timeout:
            print("IK timeout")
            return

    response = future.result()

    if response is None:
        print("IK no response")
        return

    print("IK error code:", response.error_code.val)


    if response.error_code.val != 1:
        print("IK failed")
        return

    # Extract ordered joint positions
    joint_state = response.solution.joint_state
    joint_map = dict(zip(joint_state.name, joint_state.position))

    ordered_joints = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]

    try:
        joint_positions = [joint_map[name] for name in ordered_joints]
    except KeyError:
        print("Joint mismatch")
        return

    # Send trajectory to controller
    goal_msg = FollowJointTrajectory.Goal()
    goal_msg.trajectory.joint_names = ordered_joints

    point = JointTrajectoryPoint()
    point.positions = joint_positions
    point.time_from_start = Duration(sec=2)

    goal_msg.trajectory.points.append(point)

    future = ROBOT.joint_client.send_goal_async(goal_msg)

    while not future.done():
        rclpy.spin_once(ROBOT, timeout_sec=0.1)

    goal_handle = future.result()

    if not goal_handle.accepted:
        print("Trajectory rejected ")
        return

    result_future = goal_handle.get_result_async()

    while not result_future.done():
        rclpy.spin_once(ROBOT, timeout_sec=0.1)

    print("MoveJoint executed")

    # Update internal state con la pose original
    ROBOT.current_pose = abs_xyz
    ROBOT.current_orientation = abs_ypr

    time.sleep(wait_time)


# ==============================================================
# Relative Movements
# ==============================================================

def MoveRelLinear(relative_xyz, speed, wait_time):
    """
    Performs relative Cartesian displacement
    with respect to the last commanded pose.
    """
    new_pose = [
        ROBOT.current_pose[0] + float(relative_xyz[0]),
        ROBOT.current_pose[1] + float(relative_xyz[1]),
        ROBOT.current_pose[2] + float(relative_xyz[2]),
    ]

    MoveJoint(new_pose, ROBOT.current_orientation, speed, wait_time)


def MoveRelReor(relative_ypr, speed, wait_time):
    """
    Performs relative orientation change (Euler angles)
    with respect to the last commanded orientation.
    """
    new_orientation = [
        ROBOT.current_orientation[0] + float(relative_ypr[0]),
        ROBOT.current_orientation[1] + float(relative_ypr[1]),
        ROBOT.current_orientation[2] + float(relative_ypr[2]),
    ]

    MoveJoint(ROBOT.current_pose, new_orientation, speed, wait_time)


# ==============================================================
# GRIPPER CONTROL
# ==============================================================

def GripperSet(relative_closure, wait_time):
    """
    Controls the Robotiq gripper.

    0%   = fully open
    80% = fully closed

    IMPORTANT:
    In Gazebo Harmonic, grasping is NOT done via attach plugin.
    The object is held by physical contact and friction.
    """

    # Gripper limits in simulation
    max_open = 0.8
    min_close = 0.0

    # Convert percentage to joint position
    position = max_open - (max_open - min_close) * (relative_closure / 100.0)

    goal_msg = FollowJointTrajectory.Goal()
    goal_msg.trajectory.joint_names = [
        "robotiq_85_left_knuckle_joint"
    ]

    point = JointTrajectoryPoint()
    point.positions = [position]
    point.time_from_start = Duration(sec=1)

    goal_msg.trajectory.points.append(point)

    future = ROBOT.gripper_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(ROBOT, future)
    goal_handle = future.result()

    if not goal_handle.accepted:
        print("Gripper trajectory rejected")
        return

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(ROBOT, result_future)

    # Si abrimos completamente (0%) → liberar objeto
    if relative_closure <= 5:
        dettach()

    time.sleep(wait_time)


def attach(item):
    """
    Logical attach (debug only).

    In Harmonic, no artificial joint is created.
    The grasp is purely physical.
    """
    ROBOT.grasped_object = item
    print(f"Object logically attached: {item}")


def dettach():
    """
    Logical detach (debug only).
    Real release happens when gripper opens.
    """
    if ROBOT.grasped_object is not None:
        print(f"Object logically detached: {ROBOT.grasped_object}")
        ROBOT.grasped_object = None