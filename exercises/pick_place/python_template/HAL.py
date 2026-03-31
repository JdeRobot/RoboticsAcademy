print("HAL initializing", flush=True)
##############################################################################
# JdeROBOT ROBOTICS ACADEMY (http://jderobot.github.io/RoboticsAcademy/)
#  API PICK and PLACE exercise, including:
#   Robot Info: get_TCP_pose, get_Joint_states
#   Kinematics: MoveAbsJ, MoveJoint, MoveLinear, MoveSingleJ
#               MoveRelLinear, MoveRelReor
#   Gripper: GripperSet, attach, dettach
#
#   VERSION: 1.0
# 	DATE: 	 April 21, 2025
#   AUTHOR:  Diego Martin (diego.martin.martin@gmail.com)
#
# ======= Acknowledgments =======
#  IFRA-Cranfield nice "ROS2 Sim-to-Real Robot Control" package
#  URL: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl
##############################################################################

import sys, os, time, math
import rclpy

# import tf_transformations
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import JointState
from ros2srrc_data.msg import Robpose
from linkattacher_msgs.srv import AttachLink, DetachLink
from ament_index_python.packages import get_package_share_directory

# Build PATH and import Python classes from IFRA package:
PATH = os.path.join(get_package_share_directory("ros2srrc_execution"), "python")

PATH_ROB = PATH + "/robot"
sys.path.append(PATH_ROB)
from robot import RBT

PATH_EE = PATH + "/endeffector"
sys.path.append(PATH_EE)
from robotiq_ur import RobotiqGRIPPER

# Import ROS2 Custom Messages from IFRA package:
from ros2srrc_data.msg import Action
from ros2srrc_data.msg import Joint
from ros2srrc_data.msg import Joints
from ros2srrc_data.msg import Xyz
from ros2srrc_data.msg import Ypr
from ros2srrc_data.msg import Robpose

# Inicialization
rclpy.init(args=None)
UR5 = RBT()


#################################### ROBOT KINEMATICS ###################################################
# MoveAbsJ. Absolute Joints in degrees, speed max 1.0, wait time after movement in seconds
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

    # Print movement results if movement succeeded
    if EXECUTION["Success"] == True:
        print(f"Robot moved to Joint Angular Goal: {absolute_joints}")
        print(
            f"Movement Execution Time: {EXECUTION['ExecTime']} s at Robot Speed: {speed*100} %"
        )
    else:
        print("Robot movement FAILED, check REASON in MoveIt output")

    # Wait till next movement
    time.sleep(wait_time)
    print(f"Waiting {wait_time} s")
    print("")


# MoveLinear. Linear movement to absolute pose XYZ with YPR absolute orientation in degrees
# Speed max 1.0, wait time after movement in seconds
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


# MoveJoint. Point-to-point movement to absolute pose XYZ with YPR absolute orientation in degrees
# Speed max 1.0, wait time after movement in seconds
def MoveJoint(abs_xyz, abs_ypr, speed, wait_time):

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

    EXECUTION = UR5.RobMove_EXECUTE("PTP", float(speed), InputPose)

    # Print movement results if movement succeeded
    if EXECUTION["Success"] == True:
        print(
            f"Robot moved Point-to-Point to Abs XYZ: {abs_xyz} and Abs YPR: {abs_ypr}"
        )
        print(
            f"Movement Execution Time: {EXECUTION['ExecTime']} s at Robot Speed: {speed*100} %"
        )
    else:
        print("Robot movement FAILED, check REASON in MoveIt output")

    # Wait till next movement
    time.sleep(wait_time)
    print(f"Waiting {wait_time} s")
    print("")


# MoveRelLinear. Linear movement, relative cartesian coordinates
# Speed max 1.0, wait time after movement in seconds
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

    # Print movement results if movement succeeded
    if EXECUTION["Success"] == True:
        print(f"Robot moved LINEARLY by a relative increment of : {relative_xyz}")
        print(
            f"Movement Execution Time: {EXECUTION['ExecTime']} s at Robot Speed: {speed*100} %"
        )
    else:
        print("Robot movement FAILED, check REASON in MoveIt output")


# MoveSingleJ. Relative angle in degrees, speed max 1.0, wait time after movement in seconds
def MoveSingleJ(joint_number, relative_angle, speed, wait_time):
    ACTION = Action()
    ACTION.action = "MoveR"
    ACTION.speed = float(speed)

    INPUT = Joint()
    INPUT.joint = str(joint_number)
    INPUT.value = float(relative_angle)
    ACTION.mover = INPUT

    EXECUTION = UR5.Move_EXECUTE(ACTION)

    # Print movement results if movement succeeded
    if EXECUTION["Success"] == True:
        print(f"Robot moved {joint_number} in {relative_angle} degrees")
        print(
            f"Movement Execution Time: {EXECUTION['ExecTime']} s at Robot Speed: {speed*100} %"
        )
    else:
        print("Robot movement FAILED, check REASON in MoveIt output")

    # Wait till next movement
    time.sleep(wait_time)
    print(f"Waiting {wait_time} s")
    print("")


# Relative Reorient given relative Euler Angles
# Speed max 1.0, wait time after movement in seconds
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

    # Print movement results if movement succeeded
    if EXECUTION["Success"] == True:
        print(f"TCP reoriented by a relative increment of : {relative_ypr}")
        print(
            f"Movement Execution Time: {EXECUTION['ExecTime']} s at Robot Speed: {speed*100} %"
        )
    else:
        print("Robot movement FAILED, check REASON in MoveIt output")

    # Wait till next movement
    time.sleep(wait_time)
    print(f"Waiting {wait_time} s")
    print("")


###################################### GRIPPER ###################################################
class LinkAttacherClient(Node):
    def __init__(self):
        super().__init__("link_attacher_client")
        self.attach_client = self.create_client(AttachLink, "/ATTACHLINK")
        self.detach_client = self.create_client(DetachLink, "/DETACHLINK")

        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Attach service not available, waiting again...")
        while not self.detach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Detach service not available, waiting again...")

    def send_attach_request(self, model1_name, link1_name, model2_name, link2_name):
        request = AttachLink.Request()
        request.model1_name = model1_name
        request.link1_name = link1_name
        request.model2_name = model2_name
        request.link2_name = link2_name

        future = self.attach_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_detach_request(self, model1_name, link1_name, model2_name, link2_name):
        request = DetachLink.Request()
        request.model1_name = model1_name
        request.link1_name = link1_name
        request.model2_name = model2_name
        request.link2_name = link2_name

        future = self.detach_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


# Attach object to gripper. Must be called explicitely
def attach(item):
    link_attacher_client = LinkAttacherClient()

    # Attach service call
    attach_response = link_attacher_client.send_attach_request(
        "ur5", "EE_robotiq_2f85", item, item
    )
    link_attacher_client.get_logger().info(
        "Attach Response: %s" % attach_response.success
    )


# Dettach all objects. It is always called when gripper is set to full open (0%)
def dettach():
    link_attacher_client = LinkAttacherClient()

    # Detach operation for all possible objects when gripper is set to 0%
    link_attacher_client.send_detach_request(
        "ur5", "EE_robotiq_2f85", "red_box", "red_box"
    )
    link_attacher_client.send_detach_request(
        "ur5", "EE_robotiq_2f85", "yellow_box", "yellow_box"
    )
    link_attacher_client.send_detach_request(
        "ur5", "EE_robotiq_2f85", "blue_sphere", "blue_sphere"
    )
    link_attacher_client.send_detach_request(
        "ur5", "EE_robotiq_2f85", "green_cylinder", "green_cylinder"
    )


# Set gripper closure to a given percentage (0% fully open, 100% fully closed)
# Speed max 1.0, wait time after movement in seconds
def GripperSet(relative_closure, wait_time):
    ACTION = Action()
    ACTION.action = "MoveG"
    ACTION.speed = float(1)  # Gripper speed not working for Robotiq 85, set to 100%

    ACTION.moveg = float(relative_closure)

    EXECUTION = UR5.Move_EXECUTE(ACTION)

    # Print movement results if movement succeeded
    if EXECUTION["Success"] == True:
        print(f"Gripper set to a percentage of: {relative_closure} %")
        print(f"Movement Execution Time: {EXECUTION['ExecTime']} s")
        if relative_closure == 0:
            dettach()  # Automatic object dettach from gripper when full open (0%)

    else:
        print("Gripper closing FAILED, check REASON in MoveIt output")

    # Wait till next movement
    time.sleep(wait_time)
    print(f"Waiting {wait_time} s")
    print("")
