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

class Turtlebot():
    def __init__(self):
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.play_event = Event()
        rospy.sleep(2)
        # self.stop_turtlebot()

        self.object_list = {}
        __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
        filename = os.path.join(__location__, 'models_info.yaml')
        with open(filename) as file:
            self.objects_info = yaml.load(file)

    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initRobot(cls):
        new_instance = cls()
        return new_instance

    # def start_turtlebot(self):
    #     try:
    #         self.play_event.set()
    #         rospy.sleep(0.5)
    #         self.thread.join()
    #     except:
    #         print("Thread not yet started")

    # def stop_turtlebot(self):
    #     self.play_event.clear()
    #     self.thread = StoppableThread(target=self.__stop__, args=[])
    #     self.thread.start()

    # def __stop__(self):
    #     rec = self.get_state("turtlebot3", "base_link")
    #     req = ModelState()
    #     req.model_name = "turtlebot3"
    #     req.twist.linear.x = 0.0
    #     req.twist.linear.y = 0.0
    #     req.twist.linear.z = 0.0
    #     req.twist.angular.x = 0.0
    #     req.twist.angular.y = 0.0
    #     req.twist.angular.z = 0.0
    #     req.pose.position.x = rec.pose.position.x + 3.0
    #     req.pose.position.y = rec.pose.position.y + 1.0
    #     req.pose.position.z = 0.0
    #     req.pose.orientation.x = rec.pose.orientation.x
    #     req.pose.orientation.y = rec.pose.orientation.y
    #     req.pose.orientation.z = rec.pose.orientation.z
    #     req.pose.orientation.w = 1.0
    #     while True:
    #         if self.play_event.is_set():
    #             sys.exit() # kill stop_turtlebot thread
    #         else:
    #             self.set_state(req)

    # def reset_turtlebot(self):
    #     self.start_turtlebot()
    #     req = ModelState()
    #     req.model_name = "turtlebot3"
    #     req.twist.linear.x = 0.0
    #     req.twist.linear.y = 0.0
    #     req.twist.linear.z = 0.0
    #     req.twist.angular.x = 0.0
    #     req.twist.angular.y = 0.0
    #     req.twist.angular.z = 0.0
    #     req.pose.position.x = 0.0
    #     req.pose.position.y = 0.0
    #     req.pose.position.z = 0.0
    #     req.pose.orientation.x = 0.0
    #     req.pose.orientation.y = 0.0
    #     req.pose.orientation.z = 0.0
    #     req.pose.orientation.w = 0.0
    #     self.set_state(req)
    #     self.stop_turtlebot()

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