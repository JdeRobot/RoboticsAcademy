import rospy
import rospkg
import threading
import time
from datetime import datetime

import sys
import copy
import os
import numpy

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
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion
        
# Hardware Abstraction Layer
class HAL:
    IMG_WIDTH = 320
    IMG_HEIGHT = 240
    
    def __init__(self):
        rospy.init_node("HAL")
    
        # self.image = None
        # self.drone = DroneWrapper(name="rqt")

        self.arm = moveit_commander.MoveGroupCommander("irb_120")
        self.gripper = moveit_commander.MoveGroupCommander("robotiq_85")

        self.arm.set_goal_tolerance(0.05)
        self.gripper.set_goal_tolerance(0.02)

        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()
        rospy.sleep(1)

        # self.add_objects()
        # self.add_table()
        #self.add_ground()

        self.approach_retreat_desired_dist = 0.2
        self.approach_retreat_min_dist = 0.1

        rospy.sleep(1.0)

    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initRobot(cls):
        new_instance = cls()
        return new_instance
    
    # Get Image from ROS Driver Camera
    def pickup(self, object_name, pose):
        grasps = self.generate_grasps(object_name, pose)
        self.arm.pick(object_name, grasps)
        #self.gripper.stop()

        rospy.loginfo('Pick up successfully')
        self.arm.detach_object(object_name)
        self.clean_scene(object_name)
        #rospy.sleep(1)

    def place(self, pose):
        self.move_pose_arm(pose)
        rospy.sleep(1)

        # pose.position.z -= 0.1
        # self.move_pose_arm(pose)
        
        waypoints = []
        wpose = self.arm.get_current_pose().pose
        wpose.position.z -= 0.15
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.arm.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
        self.arm.execute(plan, wait=True)

        self.move_joint_hand(0)
        rospy.sleep(1)
        
        # pose.position.z += 0.1
        # self.move_pose_arm(pose)

        waypoints = []
        wpose = self.arm.get_current_pose().pose
        wpose.position.z += 0.15
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.arm.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
        self.arm.execute(plan, wait=True)

        rospy.loginfo('Place successfully')

    def back_to_home(self):
        self.move_joint_arm(0,0,0,0,0,0)
        self.move_joint_hand(0)
        rospy.sleep(1)

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

    def move_pose_arm(self, pose_goal):
        self.arm.set_pose_target(pose_goal)

        self.arm.go(wait=True)

        self.arm.stop() # To guarantee no residual movement
        self.arm.clear_pose_targets()

    def move_joint_hand(self,gripper_finger1_joint):
        joint_goal = self.gripper.get_current_joint_values()
        joint_goal[2] = gripper_finger1_joint # Gripper master axis

        self.gripper.go(joint_goal, wait=True)
        self.gripper.stop() # To guarantee no residual movement

    def generate_grasps(self, name, pose):
        grasps = []

        now = rospy.Time.now()
        angle = 0
        grasp = Grasp()

        grasp.grasp_pose.header.stamp = now
        grasp.grasp_pose.header.frame_id = self.robot.get_planning_frame()
        grasp.grasp_pose.pose = copy.deepcopy(pose)

        # Setting pre-grasp approach
        grasp.pre_grasp_approach.direction.header.stamp = now
        grasp.pre_grasp_approach.direction.header.frame_id = self.robot.get_planning_frame()
        grasp.pre_grasp_approach.direction.vector.z = -0.5
        grasp.pre_grasp_approach.min_distance = self.approach_retreat_min_dist
        grasp.pre_grasp_approach.desired_distance = self.approach_retreat_desired_dist

        # Setting post-grasp retreat
        grasp.post_grasp_retreat.direction.header.stamp = now
        grasp.post_grasp_retreat.direction.header.frame_id = self.robot.get_planning_frame()
        grasp.post_grasp_retreat.direction.vector.z = 0.5
        grasp.post_grasp_retreat.min_distance = self.approach_retreat_min_dist
        grasp.post_grasp_retreat.desired_distance = self.approach_retreat_desired_dist


        q = quaternion_from_euler(0.0, numpy.deg2rad(90.0), angle)
        grasp.grasp_pose.pose.orientation = Quaternion(*q)

        grasp.max_contact_force = 1000

        grasp.pre_grasp_posture.joint_names.append("gripper_finger1_joint")
        traj = JointTrajectoryPoint()
        traj.positions.append(0.0)
        traj.time_from_start = rospy.Duration.from_sec(0.5)
        grasp.pre_grasp_posture.points.append(traj)

        grasp.grasp_posture.joint_names.append("gripper_finger1_joint")
        traj = JointTrajectoryPoint()
        if name == "box":
            traj.positions.append(0.4)
        elif name == "ball":
            traj.positions.append(0.3)
        elif name == "cylinder":
            traj.positions.append(0.3)

        #traj.velocities.append(0.2)
        #traj.effort.append(100)
        traj.time_from_start = rospy.Duration.from_sec(5.0)
        grasp.grasp_posture.points.append(traj)

        grasps.append(grasp)

        return grasps
