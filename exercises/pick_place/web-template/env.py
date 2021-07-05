import sys

from numpy.lib.function_base import select
import rospy, rospkg
import os
import copy
import yaml
from moveit_commander import RobotCommander,PlanningSceneInterface
import moveit_commander
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from threading import Event

from interfaces.threadStoppable import StoppableThread

class Object:
    def __init__(self, relative_pose, abs_pose, height, width, length, shape, color):
        self.relative_pose = relative_pose
        self.abs_pose = abs_pose
        self.height = height
        self.width = width
        self.length = length
        self.shape = shape
        self.color = color

class ENV():
    def __init__(self, object_list):
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        self.spawn_model_srv = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

        rospy.wait_for_service("gazebo/spawn_sdf_model")
        self.delete_model_srv = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

        self.arm = moveit_commander.MoveGroupCommander("irb_120")
        self.gripper = moveit_commander.MoveGroupCommander("robotiq_85")

        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()

        self.object_list = object_list
        self.play_event = Event()

    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initRobot(cls):
        new_instance = cls()
        return new_instance

    def reset(self):
        self.move_joint_arm(0, 0, 0, 0, 0, 0)
        self.move_joint_hand(0)
        self.play_event.clear()
        self.thread = StoppableThread(target = self.respawn_all_objects, args=[])

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

    def move_joint_hand(self,gripper_finger1_joint):
        joint_goal = self.gripper.get_current_joint_values()
        joint_goal[2] = gripper_finger1_joint # Gripper master axis

        self.gripper.go(joint_goal, wait=True)
        self.gripper.stop() # To guarantee no residual movement
    
    def pose2msg(self, x, y, z, roll, pitch, yaw):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        quat = quaternion_from_euler(roll,pitch,yaw)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
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

    def spawn_model(self, model_name, model_pose):
        with open(os.path.join(rospkg.RosPack().get_path('irb120_robotiq85_gazebo'), 'models', model_name,'model.sdf'), "r") as f:
            model_xml = f.read()

        self.spawn_model_srv(model_name, model_xml, "", model_pose, "world")

    def delete_model(self, model_name):
        self.delete_model_srv(model_name)

    def clean_scene(self, object_name):
        self.scene.remove_world_object(object_name)

    def respawn_all_objects(self):
        objects_name = self.object_list.keys()
        for object_name in objects_name:
            this_object = self.object_list[object_name]
            print("Respawning {}".format(object_name))
            # remove old objects in Gazebo
            self.delete_model(object_name)

            # respawn new objects in Gazebo
            roll, pitch, yaw, x, y, z = self.msg2pose(this_object.abs_pose)
            object_pose = self.pose2msg(x, y, z, roll, pitch, yaw)
            self.spawn_model(object_name, object_pose)

            # respawn objects in Rviz
            p = PoseStamped()
            p.header.frame_id = self.robot.get_planning_frame()
            p.header.stamp = rospy.Time.now()

            self.clean_scene(object_name)
            p.pose = copy.copy(this_object.relative_pose)
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

            # rospy.sleep(0.5)
        rospy.loginfo("All objects are respawned")
        if self.play_event.is_set():
            sys.exit()