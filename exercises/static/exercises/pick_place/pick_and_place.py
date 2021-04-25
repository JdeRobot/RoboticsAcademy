#! /usr/bin/env python

import rospy
import rospkg

import sys
import copy
import os
import numpy
import math

from moveit_commander import RobotCommander,PlanningSceneInterface
from moveit_commander import roscpp_initialize, roscpp_shutdown
from moveit_commander.conversions import pose_to_list
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import Grasp, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import String, Bool

from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix

from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion, Vector3, Point
import threading
import yaml


class Object:
    def __init__(self, relative_pose, abs_pose, height, width, length, shape, color):
        self.relative_pose = relative_pose
        self.abs_pose = abs_pose
        self.height = height
        self.width = width
        self.length = length
        self.shape = shape
        self.color = color


class WorkSpace:
    def __init__(self, x, y, z, min_r, max_r, min_z):
        self.x = x
        self.y = y
        self.z = z
        self.min_r = min_r
        self.max_r = max_r
        self.min_z = min_z


class Pick_Place:
    def __init__ (self):
        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()

        filename = os.path.join(rospkg.RosPack().get_path('rqt_industrial_robot'), 'src','rqt_kinematics', 'joints_setup.yaml')
        with open(filename) as file:
            joints_setup = yaml.load(file)
            jointslimit = joints_setup["joints_limit"]

            home_value = joints_setup["home_value"]
            j1 = home_value["joint_1"]
            j2 = home_value["joint_2"]
            j3 = home_value["joint_3"]
            j4 = home_value["joint_4"]
            j5 = home_value["joint_5"]
            j6 = home_value["joint_6"]
            g = home_value["gripper"]
            self.set_home_value([j1, j2, j3, j4, j5, j6, g])

        self.object_list = {}
        filename = os.path.join(rospkg.RosPack().get_path('rqt_industrial_robot'), 'src','rqt_kinematics', 'interfaces', 'models_info.yaml')
        with open(filename) as file:
            objects_info = yaml.load(file)
            robot_x = objects_info["robot"]["pose"]["x"]
            robot_y = objects_info["robot"]["pose"]["y"]
            robot_z = objects_info["robot"]["pose"]["z"]
            robot_roll = objects_info["robot"]["pose"]["roll"]
            robot_pitch = objects_info["robot"]["pose"]["pitch"]
            robot_yaw = objects_info["robot"]["pose"]["yaw"]

            rospy.loginfo("Spawning Objects in Gazebo and planning scene")
            objects = objects_info["objects"]
            objects_name = objects.keys()
            for object_name in objects_name:
                name = object_name
                shape = objects[name]["shape"]
                color = objects[name]["color"]

                x = objects[name]["pose"]["x"]
                y = objects[name]["pose"]["y"]
                z = objects[name]["pose"]["z"]
                roll = objects[name]["pose"]["roll"]
                pitch = objects[name]["pose"]["pitch"]
                yaw = objects[name]["pose"]["yaw"]
                object_pose = self.pose2msg(x, y, z, roll, pitch, yaw)

                p = PoseStamped()
                p.header.frame_id = self.robot.get_planning_frame()
                p.header.stamp = rospy.Time.now()

                p.pose.position.x = x - robot_x
                p.pose.position.y = y - robot_y
                p.pose.position.z = z - robot_z

                q = quaternion_from_euler(roll,pitch,yaw)
                p.pose.orientation = Quaternion(*q)

                if shape == "box":
                    x = objects[name]["size"]["x"]
                    y = objects[name]["size"]["y"]
                    z = objects[name]["size"]["z"]
                    p.pose.position.z += z/2

                    height = z
                    width = y
                    length = x
                    self.object_list[name] = Object(p.pose, object_pose, height, width, length, shape, color)

                elif shape == "cylinder":
                    height = objects[name]["size"]["height"]
                    radius = objects[name]["size"]["radius"]
                    p.pose.position.z += height/2
                    self.object_list[name] = Object(p.pose, object_pose, height, radius*2, radius*2, shape, color)

                elif shape == "sphere":
                    radius = objects[name]["size"]
                    p.pose.position.z += radius
                    self.object_list[name] = Object(p.pose, object_pose, radius*2, radius*2, radius*2, shape, color)

        # self.object_list = object_list
        self.goal_list = {}
        self.set_target_info()

        self.gripper_width = {}
        self.set_gripper_width_relationship()

        self.arm = moveit_commander.MoveGroupCommander("irb_120")
        self.gripper = moveit_commander.MoveGroupCommander("robotiq_85")

        self.arm.set_goal_tolerance(0.01)

        # set default grasp message infos
        self.set_grasp_distance(0.1, 0.2)
        self.set_grasp_direction(0, 0, -0.5)

        self.get_workspace()

        self.message_pub = rospy.Publisher("/gui_message", String, queue_size=0)
        self.updatepose_pub = rospy.Publisher("/updatepose", Bool, queue_size=0)
    
    def send_message(self, message):
        msg = String()
        msg.data = message
        self.message_pub.publish(msg)

    def updatepose_trigger(self, value):
        msg = Bool()
        msg.data = value
        self.updatepose_pub.publish(msg)

    def clean_scene(self, object_name):
        self.scene.remove_world_object(object_name)

    def set_target_info(self):
        filename = os.path.join(rospkg.RosPack().get_path('rqt_industrial_robot'), 'src','rqt_kinematics', 'interfaces', 'models_info.yaml')
        with open(filename) as file:
            objects_info = yaml.load(file)
            robot_x = objects_info["robot"]["pose"]["x"]
            robot_y = objects_info["robot"]["pose"]["y"]
            robot_z = objects_info["robot"]["pose"]["z"]

            targets = objects_info["targets"]
            target_name = targets.keys()
            for name in target_name:
                position = Point()
                position.x = targets[name]["x"] - robot_x
                position.y = targets[name]["y"] - robot_y
                position.z = targets[name]["z"] - robot_z
                self.goal_list[name] = position

    def set_gripper_width_relationship(self):
        filename = os.path.join(rospkg.RosPack().get_path('rqt_industrial_robot'), 'src','rqt_kinematics', 'interfaces', 'models_info.yaml')
        with open(filename) as file:
            objects_info = yaml.load(file)
            gripper_joint_value = objects_info["gripper_joint_value"]

            objects_width = gripper_joint_value.keys()
            for object_width in objects_width:
                self.gripper_width[object_width] = gripper_joint_value[object_width]

    def get_object_list(self):
        return self.object_list.keys()

    def get_target_list(self):
        return self.goal_list.keys()

    def get_object_pose(self, object_name):
        return copy.deepcopy(self.object_list[object_name].relative_pose)

    def get_object_info(self, object_name):
        this_object = copy.deepcopy(self.object_list[object_name])
        pose = this_object.relative_pose
        height = this_object.height
        width = this_object.width
        length = this_object.length
        shape = this_object.shape
        color = this_object.color
        return pose, height, width, length, shape, color

    def get_target_position(self, target_name):
        return self.goal_list[target_name]

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

    def pose2msg_deg(self, roll, pitch, yaw, x, y, z):
        pose = geometry_msgs.msg.Pose()
        quat = quaternion_from_euler(numpy.deg2rad(roll),numpy.deg2rad(pitch),numpy.deg2rad(yaw))
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        return pose

    def msg2pose_deg(self, pose):
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        quaternion = (pose.orientation.x,
                      pose.orientation.y,
                      pose.orientation.z,
                      pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        roll = numpy.rad2deg(euler[0])
        pitch = numpy.rad2deg(euler[1])
        yaw = numpy.rad2deg(euler[2])

        return roll, pitch, yaw, x, y, z 

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

    def set_home_value(self, home_value):
        self.home_value = home_value

    def back_to_home(self):
        j1, j2, j3, j4, j5, j6, g = self.home_value
        self.move_joint_arm(j1, j2, j3, j4, j5, j6)
        self.move_joint_hand(g)
        rospy.sleep(1)

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
        self.updatepose_trigger(True)

    # Inverse Kinematics: Move the robot arm to desired pose
    def move_pose_arm(self, pose_goal):
        position = pose_goal.position
        if not self.is_inside_workspace(position.x, position.y, position.z):
            rospy.loginfo('***** GOAL POSE IS OUT OF ROBOT WORKSPACE *****')
            return

        self.arm.set_pose_target(pose_goal)

        self.arm.go(wait=True)

        self.arm.stop() # To guarantee no residual movement
        self.arm.clear_pose_targets()
        self.updatepose_trigger(True)

    # Move the Robotiq gripper by master axis
    def move_joint_hand(self,gripper_finger1_joint):
        joint_goal = self.gripper.get_current_joint_values()
        joint_goal[2] = gripper_finger1_joint # Gripper master axis

        self.gripper.go(joint_goal, wait=True)
        self.gripper.stop() # To guarantee no residual movement
        self.updatepose_trigger(True)

    def set_grasp_direction(self, x, y, z):
        self.approach_direction = Vector3()
        self.approach_direction.x = x
        self.approach_direction.y = y
        self.approach_direction.z = z

        self.retreat_direction = Vector3()
        self.retreat_direction.x = -x
        self.retreat_direction.y = -y
        self.retreat_direction.z = -z

    def set_grasp_distance(self, min_distance, desired_distance):
        self.approach_retreat_min_dist = min_distance
        self.approach_retreat_desired_dist = desired_distance

    def count_gripper_width(self, object_name):
        object_width = self.object_list[object_name].width
        if object_width in self.gripper_width:
            return self.gripper_width[object_width]
        else:
            rospy.loginfo("Cannot find suitable gripper joint value for this object width")
            return 0

    def generate_grasp(self, object_name, eef_orientation, position, width = 0, roll = 0, pitch = 0, yaw = 0, length = 0):
        if width == 0: # need to count gripper joint value
            if (eef_orientation == "horizontal" and pitch == 0) or (eef_orientation == "vertical" and yaw == 0):
                width = self.count_gripper_width(object_name)
            else:
                rospy.loginfo("Orientation doesn't meet requirement. Please tune gripper width by yourself")

        return self.generate_grasp_width(eef_orientation, position, width, roll, pitch, yaw, length)

    def generate_grasp_width(self, eef_orientation, position, width, roll = 0, pitch = 0, yaw = 0, length = 0):
        now = rospy.Time.now()
        grasp = Grasp()

        grasp.grasp_pose.header.stamp = now
        grasp.grasp_pose.header.frame_id = self.robot.get_planning_frame()

        grasp.grasp_pose.pose.position = position

        if eef_orientation == "horizontal":
            q = quaternion_from_euler(0.0, numpy.deg2rad(pitch), 0.0)
        elif eef_orientation == "vertical":
            q = quaternion_from_euler(0.0, numpy.deg2rad(90.0), numpy.deg2rad(yaw))
        elif eef_orientation == "user_defined":
            q = quaternion_from_euler(numpy.deg2rad(roll), numpy.deg2rad(pitch), numpy.deg2rad(yaw))

        grasp.grasp_pose.pose.orientation = Quaternion(*q)

        # transform from gripper to TCP
        grasp.grasp_pose.pose = self.gripper2TCP(grasp.grasp_pose.pose, length)

        if not self.is_inside_workspace(grasp.grasp_pose.pose.position.x, grasp.grasp_pose.pose.position.y, grasp.grasp_pose.pose.position.z):
            rospy.loginfo('***** GOAL POSE IS OUT OF ROBOT WORKSPACE *****')
            #return False

        # Setting pre-grasp approach
        grasp.pre_grasp_approach.direction.header.stamp = now
        grasp.pre_grasp_approach.direction.header.frame_id = self.robot.get_planning_frame()
        grasp.pre_grasp_approach.direction.vector = self.approach_direction
        grasp.pre_grasp_approach.min_distance = self.approach_retreat_min_dist
        grasp.pre_grasp_approach.desired_distance = self.approach_retreat_desired_dist

        # Setting post-grasp retreat
        grasp.post_grasp_retreat.direction.header.stamp = now
        grasp.post_grasp_retreat.direction.header.frame_id = self.robot.get_planning_frame()
        grasp.post_grasp_retreat.direction.vector = self.retreat_direction
        grasp.post_grasp_retreat.min_distance = self.approach_retreat_min_dist
        grasp.post_grasp_retreat.desired_distance = self.approach_retreat_desired_dist

        grasp.max_contact_force = 1000

        grasp.pre_grasp_posture.joint_names.append("gripper_finger1_joint")
        traj = JointTrajectoryPoint()
        traj.positions.append(0.0)
        traj.time_from_start = rospy.Duration.from_sec(0.5)
        grasp.pre_grasp_posture.points.append(traj)

        grasp.grasp_posture.joint_names.append("gripper_finger1_joint")
        traj = JointTrajectoryPoint()
        traj.positions.append(width)

        traj.time_from_start = rospy.Duration.from_sec(5.0)
        grasp.grasp_posture.points.append(traj)

        return grasp

    # pick up object with grasps
    def pickup(self, object_name, grasps):
        rospy.loginfo('Start picking '+object_name)
        self.arm.pick(object_name, grasps)
        #self.gripper.stop()
        self.updatepose_trigger(True)

        rospy.loginfo('Pick up finished')
        self.arm.detach_object(object_name)
        self.clean_scene(object_name)

    # place object to goal position
    def place(self, eef_orientation, position, distance = 0.1, roll = 0, pitch = 0, yaw = 180):
        if not self.is_inside_workspace(position.x, position.y, position.z):
            rospy.loginfo('***** GOAL POSE IS OUT OF ROBOT WORKSPACE *****')
            rospy.loginfo('Stop placing')
            return

        pose = Pose()
        pose.position = position

        if eef_orientation == "horizontal":
            q = quaternion_from_euler(0.0, numpy.deg2rad(pitch), numpy.deg2rad(180))
        elif eef_orientation == "vertical":
            q = quaternion_from_euler(0.0, numpy.deg2rad(90.0), numpy.deg2rad(yaw))
        elif eef_orientation == "user_defined":
            q = quaternion_from_euler(numpy.deg2rad(roll), numpy.deg2rad(pitch), numpy.deg2rad(yaw))

        pose.orientation = Quaternion(*q)
        pose.position.z += distance

        rospy.loginfo('Start placing')
        self.move_pose_arm(pose)
        rospy.sleep(1)
        
        # move down
        waypoints = []
        wpose = self.arm.get_current_pose().pose
        wpose.position.z -= distance
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.arm.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
        self.arm.execute(plan, wait=True)
        self.updatepose_trigger(True)

        # place
        self.move_joint_hand(0)
        rospy.sleep(1)

        # move up
        waypoints = []
        wpose = self.arm.get_current_pose().pose
        wpose.position.z += distance
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.arm.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
        self.arm.execute(plan, wait=True)
        self.updatepose_trigger(True)

        rospy.loginfo('Place finished')

