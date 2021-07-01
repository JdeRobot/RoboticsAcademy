import sys
import rospy, rospkg
import os
import copy
import yaml
from moveit_commander import RobotCommander,PlanningSceneInterface
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from threading import Event

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
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
    def __init__(self):
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.play_event = Event()
        rospy.sleep(2)
        # self.stop_turtlebot()

        __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
        filename = os.path.join(__location__, 'models_info.yaml')
        with open(filename) as file:
            self.objects_info = yaml.load(file)
        self.get_object_list = self.objects_info["objects"]

    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initRobot(cls):
        new_instance = cls()
        return new_instance

    def get_object_pose(self, object_name):
        pose = self.object_list[object_name].pose
        return pose

    def get_object_list(self):
        objects_list = {}
        for name in self.object_list:
            objects_list.append(name)
        return objects_list

    def get_object_info(self, object_name):
        return self.object_list[object_name]

    def get_target_list(self):
        target_list = {}
        for target in self.objects_info["targets"]:
            target_list.append(target)
        return target_list

    def get_target_position(self, target_name):
        return self.objects_info["targets"][target_name]