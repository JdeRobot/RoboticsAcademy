################################
# This file contains the helper APIs used by the exercise.
################################
import rospy, rospkg
import os
import copy
import yaml
from moveit_commander import RobotCommander,PlanningSceneInterface
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

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
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        self.spawn_model_srv = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

        rospy.wait_for_service("gazebo/spawn_sdf_model")
        self.delete_model_srv = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()
        rospy.sleep(1)

        self.object_list = {}
        self.spawn_all_model()

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