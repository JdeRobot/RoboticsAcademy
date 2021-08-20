####################################
# This file is just for reference, this is not being called by exercise.py
####################################

#!/usr/bin/python
import rospy
import rospkg

import sys
import copy
import os
import numpy
import threading
import yaml
import math

from moveit_commander import RobotCommander,PlanningSceneInterface
from moveit_commander import roscpp_initialize, roscpp_shutdown
from moveit_commander.conversions import pose_to_list
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import Grasp, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import String

from math import pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix

from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion
from model_manager import ModelManager


class WorkSpace:
    def __init__(self, x, y, z, min_r, max_r, min_z):
        self.x = x
        self.y = y
        self.z = z
        self.min_r = min_r
        self.max_r = max_r
        self.min_z = min_z


class RobotWrapper:
    def __init__(self, arm="irb_120", gripper="robotiq_85"):
        roscpp_initialize(sys.argv)

        self.arm = moveit_commander.MoveGroupCommander(arm)
        self.gripper = moveit_commander.MoveGroupCommander(gripper)

        self.pose = self.arm.get_current_pose().pose
        self.x = self.pose.position.x
        self.y = self.pose.position.y
        self.z = self.pose.position.z
        quaternion = (self.pose.orientation.x,
                    self.pose.orientation.y,
                    self.pose.orientation.z,
                    self.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw = euler[2]

        self.gripper_length = 0.15

        self.event = threading.Event()

        self.movement_finish = False

        self.modelmanager = ModelManager()
        self.modelmanager.spawn_all_model()

        self.plan_trajectory = None

        self.get_workspace()

    def test_connection(self):
        print("connected with robot wrapper")

    def get_workspace(self):
        filename = os.path.join(rospkg.RosPack().get_path('rqt_industrial_robot'), 'src','rqt_kinematics', 'joints_setup.yaml')
        with open(filename) as file:
            joints_setup = yaml.load(file)
            workspace = joints_setup["workspace"]

            x = workspace["center"]["x"]
            y = workspace["center"]["y"]
            z = workspace["center"]["z"]
            min_r = workspace["r"]["min"]
            max_r = workspace["r"]["max"]
            min_z = workspace["min_z"]
            self.workspace = WorkSpace(x, y, z, min_r, max_r, min_z)

    # check if the position is inside workspace
    def is_inside_workspace(self, x, y, z):
        if z > self.workspace.min_z:
            dx = x - self.workspace.x
            dy = y - self.workspace.y
            dz = z - self.workspace.z
            r = math.sqrt(dx**2+dy**2+dz**2)
            if self.workspace.min_r < r < self.workspace.max_r:
                return True
        
        return False

    # move one joint of the arm to value
    def set_arm_joint(self, joint_id, value):
        joint_goal = self.arm.get_current_joint_values()
        joint_goal[joint_id-1] = value
        self.arm.go(joint_goal, wait=True)
        self.arm.stop()

    # Forward Kinematics (FK): move the arm by axis values
    def move_joint_arm(self,joint_0,joint_1,joint_2,joint_3,joint_4,joint_5):
        joint_goal = self.arm.get_current_joint_values()
        joint_goal[0] = joint_0
        joint_goal[1] = joint_1
        joint_goal[2] = joint_2
        joint_goal[3] = joint_3
        joint_goal[4] = joint_4
        joint_goal[5] = joint_5

        self.arm.go(joint_goal, wait=True)
        self.arm.stop() # To guarantee no residual movement

    def get_joint_value(self, joint_id):
        joints = self.arm.get_current_joint_values()
        return joints[joint_id-1]

    def get_joints_value(self):
        joints = self.arm.get_current_joint_values()
        return joints

    def set_x(self, value):
        self.x = value
    
    def set_y(self, value):
        self.y = value

    def set_z(self, value):
        self.z = value

    def set_roll(self, value):
        self.roll = value
    
    def set_pitch(self, value):
        self.pitch = value

    def set_yaw(self, value):
        self.yaw = value

    def get_arm_position(self):
        pose = self.arm.get_current_pose().pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        return x, y, z

    def get_arm_orientation(self):
        pose = self.arm.get_current_pose().pose
        quaternion = (pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        return roll, pitch, yaw

    def get_arm_pose(self):
        pose = self.arm.get_current_pose().pose
        pose = self.TCP2gripper(pose, self.gripper_length)
        return self.msg2pose(pose)

    def set_random_pose(self):
        self.arm.set_random_target()

    def set_gripper_length(self, length):
        self.gripper_length = length

    def set_home_value(self, home_value):
        self.home_value = home_value

    def back_to_home(self):
        j1, j2, j3, j4, j5, j6, g = self.home_value
        self.move_joint_arm(j1, j2, j3, j4, j5, j6)
        self.move_joint_hand(g)

    def gripper2TCP(self, pose, length=0):
        roll, pitch, yaw, x, y, z = self.msg2pose(pose)

        T = euler_matrix(roll, pitch, yaw)
        T[0:3, 3] = numpy.array([x, y, z])

        pos_gripper_tcp = numpy.array([-length, 0, 0, 1])
        pos_tcp = T.dot(pos_gripper_tcp)
        pose.position.x = pos_tcp[0]
        pose.position.y = pos_tcp[1]
        pose.position.z = pos_tcp[2]

        return pose

    def TCP2gripper(self, pose, length=0):
        roll, pitch, yaw, x, y, z = self.msg2pose(pose)

        T = euler_matrix(roll, pitch, yaw)
        T[0:3, 3] = numpy.array([x, y, z])

        pos_gripper_tcp = numpy.array([length, 0, 0, 1])
        pos_tcp = T.dot(pos_gripper_tcp)
        pose.position.x = pos_tcp[0]
        pose.position.y = pos_tcp[1]
        pose.position.z = pos_tcp[2]

        return pose
        
    # Inverse Kinematics (IK): move TCP to given position and orientation
    def move_pose_arm(self, roll, pitch, yaw, x, y, z):
        pose_goal = self.pose2msg(roll, pitch, yaw, x, y, z)
        pose_goal = self.gripper2TCP(pose_goal, self.gripper_length)

        x = pose_goal.position.x
        y = pose_goal.position.y
        z = pose_goal.position.z

        if not self.is_inside_workspace(x, y, z):
            rospy.loginfo('***** GOAL POSE IS OUT OF ROBOT WORKSPACE *****')
            return False

        self.arm.set_pose_target(pose_goal)

        self.arm.go(wait=True)

        self.arm.stop() # To guarantee no residual movement
        self.arm.clear_pose_targets()

        return True

    # Move the Robotiq gripper by master axis
    def move_joint_hand(self,gripper_finger1_joint):
        joint_goal = self.gripper.get_current_joint_values()
        joint_goal[2] = gripper_finger1_joint # Gripper master axis

        self.gripper.go(joint_goal, wait=True)
        self.gripper.stop() # To guarantee no residual movement

    def get_gripper_joint_value(self):
        joints = self.gripper.get_current_joint_values()
        return joints[2]

    # stop move groups execution
    def stop_execution(self):
        self.arm.stop()
        self.gripper.stop()

    def stop(self):
        roscpp_shutdown()

    def pose2msg(self, roll, pitch, yaw, x, y, z):
        pose = geometry_msgs.msg.Pose()
        quat = quaternion_from_euler(roll,pitch,yaw)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        return pose

    def msg2pose(self, pose):
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        quaternion = (pose.orientation.x,
                      pose.orientation.y,
                      pose.orientation.z,
                      pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        return roll, pitch, yaw, x, y, z 

    def plan(self):
        pose_goal = self.pose2msg(self.roll, self.pitch, self.yaw, self.x, self.y, self.z)
        pose_goal = self.gripper2TCP(pose_goal, self.gripper_length)

        x = pose_goal.position.x
        y = pose_goal.position.y
        z = pose_goal.position.z

        if not self.is_inside_workspace(x, y, z):
            rospy.loginfo('***** GOAL POSE IS OUT OF ROBOT WORKSPACE *****')
            return False

        self.arm.set_pose_target(pose_goal)
        self.plan_trajectory = self.arm.plan()
        return True

    def execute(self):
        if self.plan_trajectory is not None:
            self.arm.execute(self.plan_trajectory, wait=False)
            self.plan_trajectory = None
        else:
            rospy.loginfo("No motion plan found")

        self.arm.stop() # To guarantee no residual movement
        self.arm.clear_pose_targets()