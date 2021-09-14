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
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from math import pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix

from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion, Vector3, Point
import threading
import yaml

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

class Obstacle:
    def __init__(self, relative_pose, abs_pose, height, width, length):
        self.relative_pose = relative_pose
        self.abs_pose = abs_pose
        self.height = height
        self.width = width
        self.length = length
        
# Hardware Abstraction Layer
class HAL:
    # IMG_WIDTH = 320
    # IMG_HEIGHT = 240
    
    def __init__(self):
        rospy.init_node("HAL")
        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()

        __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
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
            g = home_value["gripper"]
            self.set_home_value([j1, j2, j3, j4, j5, j6, g])

            home_value = joints_setup["pick_place_home_value"]
            j1 = home_value["joint_1"]
            j2 = home_value["joint_2"]
            j3 = home_value["joint_3"]
            j4 = home_value["joint_4"]
            j5 = home_value["joint_5"]
            j6 = home_value["joint_6"]
            g = home_value["gripper"]
            self.set_pick_place_home_value([j1, j2, j3, j4, j5, j6, g])

        self.object_list = {}
        self.obstacle_list = {}
        filename = os.path.join(__location__, 'models_info.yaml')
        with open(filename) as file:
            objects_info = yaml.load(file)

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

                p.pose.position.x = x
                p.pose.position.y = y
                p.pose.position.z = z

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
                    self.object_list[name] = Object(p.pose, p.pose, height, width, length, shape, color)

                elif shape == "cylinder":
                    height = objects[name]["size"]["height"]
                    radius = objects[name]["size"]["radius"]
                    p.pose.position.z += height/2
                    self.object_list[name] = Object(p.pose, p.pose, height, radius*2, radius*2, shape, color)

                elif shape == "sphere":
                    radius = objects[name]["size"]
                    p.pose.position.z += radius
                    self.object_list[name] = Object(p.pose, p.pose, radius*2, radius*2, radius*2, shape, color)

                objects = objects_info["objects"]
            
            obstacles = objects_info["obstacles"]
            obstacles_name = obstacles.keys()
            for object_name in obstacles_name:
                name = object_name

                x = obstacles[name]["pose"]["x"]
                y = obstacles[name]["pose"]["y"]
                z = obstacles[name]["pose"]["z"]
                roll = obstacles[name]["pose"]["roll"]
                pitch = obstacles[name]["pose"]["pitch"]
                yaw = obstacles[name]["pose"]["yaw"]
                object_pose = self.pose2msg(x, y, z, roll, pitch, yaw)

                p = PoseStamped()
                p.header.frame_id = self.robot.get_planning_frame()
                p.header.stamp = rospy.Time.now()

                p.pose.position.x = x
                p.pose.position.y = y
                p.pose.position.z = z

                x = obstacles[name]["size"]["x"]
                y = obstacles[name]["size"]["y"]
                z = obstacles[name]["size"]["z"]
                p.pose.position.z += z/2

                height = z
                width = y
                length = x
                self.obstacle_list[name] = Obstacle(p.pose, p.pose, height, width, length)

        # self.object_list = object_list
        self.goal_list = {}
        self.set_target_info()

        self.gripper_width = {}
        self.set_gripper_width_relationship()

        self.arm = moveit_commander.MoveGroupCommander("ur10_manipulator")
        self.arm.set_goal_tolerance(0.01)
        self.arm.set_pose_reference_frame("ur10_base_link")

        self.gripperpub = rospy.Publisher("gripper_controller/command", JointTrajectory, queue_size=0)

        self.transform_arm_to_baselink = Point()
        self.get_arm_to_baselink()

        self.gripper_length = 0.33

        self.get_workspace()

        self.message_pub = rospy.Publisher("/gui_message", String, queue_size=0)
        self.updatepose_pub = rospy.Publisher("/updatepose", Bool, queue_size=0)

        self.robot_pose = Pose()
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.robot_pose_callback)
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.target_pose = {}
        # print("hello")

        filename = os.path.join(__location__, 'navigation.yaml')
        with open(filename) as file:
            navigation_params = yaml.load(file)
            stop_pose = navigation_params["stop_pose"]
            target_names = stop_pose.keys()
            for target_name in target_names:
                pose = stop_pose[target_name]
                self.target_pose[target_name] = [pose["x"], pose["y"], pose["theta"]]


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

    def clean_scene(self, object_name):
        self.scene.remove_world_object(object_name)

    def clean_all_objects_in_scene(self):
        objects_name = self.object_list.keys()
        for object_name in objects_name:
            self.clean_scene(object_name)

    def get_target_pose(self, target_name):
        return self.target_pose[target_name]

    def send_goal_to_client(self, pose):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pose[0]
        goal.target_pose.pose.position.y = pose[1]
        quat = quaternion_from_euler(0,0,pose[2])
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        self.client.send_goal(goal)

    def get_result_from_client(self):
        wait = self.client.wait_for_result()
        return wait
        # if not wait:
        #     rospy.logerr("Action server not available!")
        #     rospy.signal_shutdown("Action server not available!")
        # else:
        #     print("self.client.get_result()",self.client.get_result())
        #     return self.client.get_result()

    def spawn_all_objects(self):
        objects_name = self.object_list.keys()
        for object_name in objects_name:
            self.spawn_object_rviz(object_name)

    def spawn_object_rviz(self, object_name):
        self.clean_scene(object_name)
        this_object = self.object_list[object_name]
        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()
        p.pose = copy.deepcopy(this_object.abs_pose)
        robot_pose = self.get_robot_pose()

        p.pose.position.x -= robot_pose.position.x
        p.pose.position.y -= robot_pose.position.y

        quaternion = (robot_pose.orientation.x,
                      robot_pose.orientation.y,
                      robot_pose.orientation.z,
                      robot_pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = -euler[2]
        quat = quaternion_from_euler(roll, pitch, yaw)
        p.pose.orientation.x = quat[0]
        p.pose.orientation.y = quat[1]
        p.pose.orientation.z = quat[2]
        p.pose.orientation.w = quat[3]

        self.object_list[object_name].relative_pose = self.baselink2arm(p.pose)
        shape = this_object.shape

        if shape == "box":
            x = this_object.length
            y = this_object.width
            z = this_object.height
            size = (x, y, z)
            self.scene.add_box(object_name, p, size)

        elif shape == "cylinder":
            height = this_object.height
            radius = this_object.width/2
            self.scene.add_cylinder(object_name, p, height, radius)

        elif shape == "sphere":
            radius = this_object.width/2
            self.scene.add_sphere(object_name, p, radius)

        rospy.sleep(0.5)

    def spawn_obstacle_rviz(self, obstacle_name):
        self.clean_scene(obstacle_name)
        this_object = self.obstacle_list[obstacle_name]

        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()
        p.pose = copy.deepcopy(this_object.abs_pose)
        robot_pose = self.get_robot_pose()

        # p.pose.position.x -= robot_pose.position.x
        # p.pose.position.y -= robot_pose.position.y

        quaternion = (robot_pose.orientation.x,
                      robot_pose.orientation.y,
                      robot_pose.orientation.z,
                      robot_pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = -euler[2]
        quat = quaternion_from_euler(roll, pitch, yaw)
        p.pose.orientation.x = quat[0]
        p.pose.orientation.y = quat[1]
        p.pose.orientation.z = quat[2]
        p.pose.orientation.w = quat[3]

        x = p.pose.position.x - robot_pose.position.x
        y = p.pose.position.y - robot_pose.position.y
        p.pose.position.x = math.cos(yaw)*x - math.sin(yaw)*y
        p.pose.position.y = math.sin(yaw)*x + math.cos(yaw)*y

        x = this_object.length
        y = this_object.width
        z = this_object.height
        size = (x, y, z)
        self.scene.add_box(obstacle_name, p, size)

        rospy.sleep(0.5)

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

    def set_gripper_width_relationship(self):
        __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
        filename = os.path.join(__location__, 'models_info.yaml')
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
        position = copy.deepcopy(self.goal_list[target_name])
        # print("before transformation",position)

        p = Pose()
        p.position = position
        

        robot_pose = self.get_robot_pose()

        quaternion = (robot_pose.orientation.x,
                      robot_pose.orientation.y,
                      robot_pose.orientation.z,
                      robot_pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        yaw = -euler[2]

        x = position.x - robot_pose.position.x
        y = position.y - robot_pose.position.y

        position.x = math.cos(yaw)*x - math.sin(yaw)*y
        position.y = math.sin(yaw)*x + math.cos(yaw)*y
        # print("after transformation",position, yaw)

        position = self.baselink2arm(p).position

        return position

    def robot_pose_callback(self, msg):
        self.robot_pose.position = msg.pose.pose.position
        self.robot_pose.orientation = msg.pose.pose.orientation

    def get_robot_pose(self):
        # try:
        #     listener = tf.TransformListener()
        #     (trans,rot) = listener.lookupTransform('/map', "/base_link", rospy.Time(0))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     rospy.loginfo("no transformation")
        #     return

        # pose = Pose()
        # pose.position.x = trans[0]
        # pose.position.y = trans[1]
        # pose.position.z = trans[2]
        # pose.orientation.x = rot[0]
        # pose.orientation.y = rot[1]
        # pose.orientation.z = rot[2]
        # pose.orientation.w = rot[3]
        return self.robot_pose

    def get_arm_to_baselink(self):
        # try:
        #     listener = tf.TransformListener()
        #     (trans,rot) = listener.lookupTransform('/base_link', "/ur10_base_link", rospy.Time(0))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     rospy.loginfo("no transformation")
        #     return
        
        # self.transform_arm_to_baselink.x = trans[0]
        # self.transform_arm_to_baselink.y = trans[1]
        # self.transform_arm_to_baselink.z = trans[2]
        # print(self.transform_arm_to_baselink)

        self.transform_arm_to_baselink.x = 0.205
        self.transform_arm_to_baselink.y = 0
        self.transform_arm_to_baselink.z = 0.802

    def get_workspace(self):
        __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
        filename = os.path.join(__location__, 'joints_setup.yaml')
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

    def get_arm_pose(self):
        pose = self.arm.get_current_pose().pose
        pose = self.baselink2arm(pose)
        pose = self.TCP2gripper(pose, self.gripper_length)
        # print(pose)
        return self.msg2pose(pose)

    def set_random_pose(self):
        self.arm.set_random_target()

    def set_gripper_length(self, length):
        self.gripper_length = length

    def set_home_value(self, home_value):
        self.home_value = home_value

    def back_to_home(self, move_gripper = True):
        j1, j2, j3, j4, j5, j6, g = self.home_value
        self.move_joint_arm(j1, j2, j3, j4, j5, j6)
        if move_gripper:
            self.move_joint_hand(g)

    def set_pick_place_home_value(self, home_value):
        self.pick_place_home_value = home_value

    def move_to_pick_place_home(self, move_gripper = True):
        j1, j2, j3, j4, j5, j6, g = self.pick_place_home_value
        self.move_joint_arm(j1, j2, j3, j4, j5, j6)
        if move_gripper:
            self.move_joint_hand(g)

    def fold_robot_arm(self,):
        self.move_joint_arm(0, 0, 0, 0, 0, 0)
        self.move_joint_hand(0)

    def gripper2TCP(self, pose, length=0):
        roll, pitch, yaw, x, y, z = self.msg2pose(pose)

        T = euler_matrix(roll, pitch, yaw)
        T[0:3, 3] = numpy.array([x, y, z])

        pos_gripper_tcp = numpy.array([0, -length, 0, 1])
        pos_tcp = T.dot(pos_gripper_tcp)
        pose.position.x = pos_tcp[0]
        pose.position.y = pos_tcp[1]
        pose.position.z = pos_tcp[2]

        return pose

    def TCP2gripper(self, pose, length=0):
        roll, pitch, yaw, x, y, z = self.msg2pose(pose)

        T = euler_matrix(roll, pitch, yaw)
        T[0:3, 3] = numpy.array([x, y, z])

        pos_gripper_tcp = numpy.array([0, length, 0, 1])
        pos_tcp = T.dot(pos_gripper_tcp)
        pose.position.x = pos_tcp[0]
        pose.position.y = pos_tcp[1]
        pose.position.z = pos_tcp[2]

        return pose

    def arm2baselink(self, in_pose):
        pose = copy.deepcopy(in_pose)
        pose.position.x += self.transform_arm_to_baselink.x
        pose.position.y += self.transform_arm_to_baselink.y
        pose.position.z += self.transform_arm_to_baselink.z
        return pose

    def baselink2arm(self, in_pose):
        pose = copy.deepcopy(in_pose)
        pose.position.x -= self.transform_arm_to_baselink.x
        pose.position.y -= self.transform_arm_to_baselink.y
        pose.position.z -= self.transform_arm_to_baselink.z
        return pose
        
    # Inverse Kinematics (IK): move TCP to given position and orientation
    def move_pose_arm(self, pose_goal):
        # pose_goal = self.pose2msg(roll, pitch, yaw, x, y, z)
        # pose_goal = self.gripper2TCP(pose_goal, self.gripper_length)

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

    # Move gripper
    def move_joint_hand(self,finger1_goal, finger2_goal = 10, finger3_goal = 10):
        if finger2_goal == 10 and finger3_goal == 10:
            finger2_goal, finger3_goal = finger1_goal, finger1_goal

        jointtrajectory = JointTrajectory()
        jointtrajectory.header.stamp = rospy.Time.now()
        jointtrajectory.joint_names.extend(["H1_F1J3", "H1_F2J3", "H1_F3J3"])

        joint = JointTrajectoryPoint()
        joint.positions.extend([finger1_goal, finger2_goal, finger3_goal])
        joint.time_from_start = rospy.Duration(1)
        jointtrajectory.points.append(joint)

        self.gripperpub.publish(jointtrajectory)

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

    # pick object to goal position
    def pickup(self, object_name, position, width, distance = 0.12):
        if not self.is_inside_workspace(position.x, position.y, position.z):
            rospy.loginfo('***** GOAL POSE IS OUT OF ROBOT WORKSPACE *****')
            rospy.loginfo('Stop placing')
            return

        pose = Pose()
        pose.position = position
        q = quaternion_from_euler(numpy.deg2rad(-90), 0, numpy.deg2rad(180))
        pose.orientation = Quaternion(*q)

        # transform from gripper to TCP
        pose = self.gripper2TCP(pose, self.gripper_length)

        pose.position.z += distance

        rospy.loginfo('Start picking')
        self.move_pose_arm(pose)
        rospy.sleep(1)
        
        # move down
        waypoints = []
        wpose = self.arm.get_current_pose().pose
        wpose = self.baselink2arm(wpose)
        print(wpose)
        wpose.position.z -= distance
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.arm.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
        self.arm.execute(plan, wait=True)
        self.updatepose_trigger(True)

        # pick
        rospy.sleep(0.5)
        self.move_joint_hand(width)
        rospy.sleep(3)
        # if object_name != "yellow_ball":
        #     self.arm.attach_object(object_name)

        # move up
        waypoints = []
        wpose = self.arm.get_current_pose().pose
        wpose = self.baselink2arm(wpose)
        wpose.position.z += distance
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.arm.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
        self.arm.execute(plan, wait=True)
        self.updatepose_trigger(True)

        rospy.loginfo('Pick finished')

    # place object to goal position
    def place(self, object_name, position, width = -0.2, distance = 0.12):
        if not self.is_inside_workspace(position.x, position.y, position.z):
            rospy.loginfo('***** GOAL POSE IS OUT OF ROBOT WORKSPACE *****')
            rospy.loginfo('Stop placing')
            return

        pose = Pose()
        pose.position = position
        q = quaternion_from_euler(numpy.deg2rad(-90), 0, numpy.deg2rad(180))
        pose.orientation = Quaternion(*q)

        # transform from gripper to TCP
        pose = self.gripper2TCP(pose, self.gripper_length)

        pose.position.z += distance

        rospy.loginfo('Start placing')
        self.move_pose_arm(pose)
        rospy.sleep(1)
        
        # move down
        waypoints = []
        wpose = self.arm.get_current_pose().pose
        wpose = self.baselink2arm(wpose)
        print(wpose)
        wpose.position.z -= distance
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.arm.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
        self.arm.execute(plan, wait=True)
        self.updatepose_trigger(True)

        # place
        self.move_joint_hand(width)
        rospy.sleep(3)
        # if object_name != "yellow_ball":
        #     self.arm.detach_object(object_name)
        # self.clean_scene(object_name)
        self.object_list.pop(object_name)

        # move up
        waypoints = []
        wpose = self.arm.get_current_pose().pose
        wpose = self.baselink2arm(wpose)
        wpose.position.z += distance
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.arm.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
        self.arm.execute(plan, wait=True)
        self.updatepose_trigger(True)

        rospy.loginfo('Place finished')