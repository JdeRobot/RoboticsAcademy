import rospy
import rospkg
import threading
import time
from datetime import datetime
import yaml
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
from moveit_msgs.msg import Constraints, OrientationConstraint, JointConstraint
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry

from math import pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix
import tf

from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion, Vector3, Point
import threading
import yaml

from pcl_filter_msgs.msg import ColorFilter, ShapeFilter

# from drone_wrapper import DroneWrapper
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
        
# Hardware Abstraction Layer
class HAL:
    # IMG_WIDTH = 320
    # IMG_HEIGHT = 240
    
    def __init__(self):
        rospy.init_node("HAL")
        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()

        self.object_list = {}
        __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
        filename = os.path.join(__location__, 'models_info.yaml')
        with open(filename) as file:
            objects_info = yaml.load(file)
            robot_x = objects_info["robot"]["pose"]["x"]
            robot_y = objects_info["robot"]["pose"]["y"]
            robot_z = objects_info["robot"]["pose"]["z"]
            robot_roll = objects_info["robot"]["pose"]["roll"]
            robot_pitch = objects_info["robot"]["pose"]["pitch"]
            robot_yaw = objects_info["robot"]["pose"]["yaw"]

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

        self.goal_list = {}
        self.set_target_info()

        self.arm = moveit_commander.MoveGroupCommander("ur5_manipulator")
        self.arm.set_max_velocity_scaling_factor(0.5)
        self.arm.set_goal_tolerance(0.01)

        self.gripper_pub = rospy.Publisher("/ur5/vacuum_gripper/grasp", Bool, queue_size=0)

        self.get_workspace()

        self.listener = tf.TransformListener()

        self.message_pub = rospy.Publisher("/gui_message", String, queue_size=0)
        self.updatepose_pub = rospy.Publisher("/updatepose", Bool, queue_size=0)

        self.color_filter_pub = rospy.Publisher("/start_color_filter", ColorFilter, queue_size=0)
        self.shape_filter_pub = rospy.Publisher("/start_shape_filter", ShapeFilter, queue_size=0)

        filename = os.path.join(__location__, 'joints_setup.yaml')
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
            self.set_home_value([j1, j2, j3, j4, j5, j6])

        self.color_shape_converter = {}
        self.color_shape_converter["red"] = 1
        self.color_shape_converter["green"] = 2
        self.color_shape_converter["blue"] = 3
        self.color_shape_converter["yellow"] = 4
        self.color_shape_converter["sphere"] = 1
        self.color_shape_converter["cylinder"] = 2

    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initRobot(cls):
        new_instance = cls()
        return new_instance

    def send_message(self, message):
        msg = String()
        msg.data = message
        self.message_pub.publish(msg)

    def updatepose_trigger(self, value):
        msg = Bool()
        msg.data = value
        self.updatepose_pub.publish(msg)

    def start_color_filter(self, color, rmax, rmin, gmax, gmin, bmax, bmin):
        color_filter = ColorFilter()
        color_filter.color = self.color_shape_converter[color]
        color_filter.rmax = rmax
        color_filter.rmin = rmin
        color_filter.gmax = gmax
        color_filter.gmin = gmin
        color_filter.bmax = bmax
        color_filter.bmin = bmin
        color_filter.status = True
        self.color_filter_pub.publish(color_filter)

    def stop_color_filter(self, color):
        color_filter = ColorFilter()
        color_filter.color = self.color_shape_converter[color]
        color_filter.status = False
        self.color_filter_pub.publish(color_filter)

    def start_shape_filter(self, color, shape, radius):
        shape_filter = ShapeFilter()
        shape_filter.color = self.color_shape_converter[color]
        shape_filter.shape = self.color_shape_converter[shape]
        shape_filter.radius = radius
        shape_filter.status = True
        self.shape_filter_pub.publish(shape_filter)

    def stop_shape_filter(self, color, shape):
        shape_filter = ShapeFilter()
        shape_filter.color = self.color_shape_converter[color]
        shape_filter.shape = self.color_shape_converter[shape]
        shape_filter.status = False
        self.shape_filter_pub.publish(shape_filter)

    def clean_scene(self, object_name):
        self.scene.remove_world_object(object_name)

    def set_target_info(self):
        __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
        filename = os.path.join(__location__, 'models_info.yaml')
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

    def get_object_list(self):
        return self.object_list.keys()

    def get_target_list(self):
        return self.goal_list.keys()

    def get_object_pose(self, object_name):
        return copy.deepcopy(self.object_list[object_name].relative_pose)

    def get_object_info(self, object_name):
        this_object = copy.deepcopy(self.object_list[object_name])
        height = this_object.height
        width = this_object.width
        length = this_object.length
        shape = this_object.shape
        color = this_object.color
        return height, width, length, shape, color

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
        __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
        filename = os.path.join(__location__, 'joint_setup.yaml')
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
        self.arm.set_max_velocity_scaling_factor(0.9)
        j1, j2, j3, j4, j5, j6 = self.home_value
        self.move_joint_arm(j1, j2, j3, j4, j5, j6)
        self.gripper_release()
        # self.send_message("Back to home")

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

    def gripper_grasp(self):
        msg = Bool()
        msg.data = True
        self.gripper_pub.publish(msg)

    def gripper_release(self):
        msg = Bool()
        msg.data = False
        self.gripper_pub.publish(msg)

    def get_object_position(self, object_name):
        frame_id = object_name
        try:
            (trans,rot) = self.listener.lookupTransform('/world', frame_id, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Cannot find desired object")
            return
        
        position = Point()
        position.x = trans[0]
        position.y = trans[1]
        position.z = trans[2]
        print("*************************************")
        print("Detected position of "+object_name+":")
        print(trans)
        print("*************************************")
        return position

    def buildmap(self):
        self.send_message("Building map")
        self.back_to_home()
        self.move_joint_arm(3.2, -1.57, 0.0, 0.0, -1.0, 0.0)
        rospy.sleep(1)
        self.back_to_home()

    # pick up object
    def pickup(self, object_name, position, distance = 0.2):
        if not self.is_inside_workspace(position.x, position.y, position.z):
            rospy.loginfo('***** GOAL POSE IS OUT OF ROBOT WORKSPACE *****')
            rospy.loginfo('Stop placing')
            return

        pose = Pose()
        pose.position = position

        q = quaternion_from_euler(0.0, numpy.deg2rad(90.0), 0.0)
        pose.orientation = Quaternion(*q)
        pose.position.z += distance

        constraint = Constraints()

        joint_constraint = JointConstraint()
        joint_constraint.joint_name = "shoulder_pan_joint"
        joint_constraint.position = 0
        joint_constraint.tolerance_above = 1.57
        joint_constraint.tolerance_below = 1.57
        joint_constraint.weight = 0.1
        constraint.joint_constraints.append(joint_constraint)

        self.arm.set_path_constraints(constraint)

        rospy.loginfo('Start picking')
        # self.send_message('Start picking')
        self.arm.set_max_velocity_scaling_factor(0.5)
        self.move_pose_arm(pose)
        rospy.sleep(1)
        self.arm.clear_path_constraints()
        
        # move down
        waypoints = []
        wpose = self.arm.get_current_pose().pose
        wpose.position.z -= distance
        # wpose = copy.deepcopy(pose)
        # wpose.position = position
        waypoints.append(copy.deepcopy(wpose))
        rospy.loginfo(wpose.position)

        (plan, fraction) = self.arm.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
        self.arm.execute(plan, wait=True)
        # self.move_pose_arm(wpose)
        rospy.sleep(1)

        # pick
        self.gripper_grasp()
        self.arm.attach_object(object_name)
        rospy.sleep(1)

        # move up
        waypoints = []
        wpose = self.arm.get_current_pose().pose
        wpose.position.z += distance
        waypoints.append(copy.deepcopy(wpose))

        self.arm.set_max_velocity_scaling_factor(0.2)
        (plan, fraction) = self.arm.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
        self.arm.execute(plan, wait=True)

        self.arm.clear_path_constraints()

        rospy.loginfo('Pick finished')
        # self.send_message('Pick finished')

    def get_joints_value(self):
        joints = self.arm.get_current_joint_values()
        return joints

    # place object to goal position
    def place(self, object_name, position, distance = 0.1):
        if not self.is_inside_workspace(position.x, position.y, position.z):
            rospy.loginfo('***** GOAL POSE IS OUT OF ROBOT WORKSPACE *****')
            rospy.loginfo('Stop placing')
            return

        pose = Pose()
        pose.position = position

        q = quaternion_from_euler(0.0, numpy.deg2rad(90.0), numpy.deg2rad(180.0))
        pose.orientation = Quaternion(*q)
        pose.position.z += distance

        # Add contraint
        constraint = Constraints()

        joints = self.get_joints_value()
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = "wrist_2_joint"
        joint_constraint.position = joints[4]
        joint_constraint.tolerance_above = 0.8
        joint_constraint.tolerance_below = 0.8
        joint_constraint.weight = 0.2
        constraint.joint_constraints.append(joint_constraint)

        joint_constraint = JointConstraint()
        joint_constraint.joint_name = "wrist_3_joint"
        joint_constraint.position = joints[5]
        joint_constraint.tolerance_above = 0.8
        joint_constraint.tolerance_below = 0.8
        joint_constraint.weight = 0.2
        constraint.joint_constraints.append(joint_constraint)

        joint_constraint = JointConstraint()
        joint_constraint.joint_name = "wrist_1_joint"
        joint_constraint.position = joints[3]
        joint_constraint.tolerance_above = 1.0
        joint_constraint.tolerance_below = 1.0
        joint_constraint.weight = 0.1
        constraint.joint_constraints.append(joint_constraint)

        joint_constraint = JointConstraint()
        joint_constraint.joint_name = "elbow_joint"
        joint_constraint.position = joints[2]
        joint_constraint.tolerance_above = 1.2
        joint_constraint.tolerance_below = 1.2
        joint_constraint.weight = 0.1
        constraint.joint_constraints.append(joint_constraint)

        orientation_constraint = OrientationConstraint()
        now = rospy.Time.now()
        orientation_constraint.header.stamp = now
        orientation_constraint.header.frame_id = "base_link"

        orientation_constraint.link_name = "ee_link"
        orientation_constraint.orientation = pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 10
        orientation_constraint.absolute_y_axis_tolerance = 1.0
        orientation_constraint.absolute_z_axis_tolerance = 10
        orientation_constraint.weight = 0.4
        constraint.orientation_constraints.append(orientation_constraint)

        self.arm.set_path_constraints(constraint)
        # self.arm.set_goal_position_tolerance(0.015)

        rospy.loginfo('Start placing')
        # self.send_message('Start placing')
        self.arm.set_max_velocity_scaling_factor(0.2)
        self.arm.set_workspace([-0.8, -0.8, -0.2, 0.8, 0.8, 0.3])

        self.move_pose_arm(pose)
        rospy.sleep(1)

        self.arm.clear_path_constraints()
        # self.arm.set_goal_tolerance(0.01)
        
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

        # place
        self.gripper_release()
        self.arm.detach_object(object_name)
        self.clean_scene(object_name)
        rospy.sleep(0.5)

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

        rospy.loginfo('Place finished')
        # self.send_message('Place finished')

        self.arm.set_workspace([-1, -1, -0.2, 1, 1, 1.0])
