import rospy
import cv2
import threading
import time
import os
import yaml
from datetime import datetime

from interfaces.camera import ListenerCamera
from interfaces.pose3d import ListenerPose3d
from interfaces.motors import PublisherMotors
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

# Hardware Abstraction Layer


class HAL:
    IMG_WIDTH = 320
    IMG_HEIGHT = 240

    def __init__(self):
        rospy.init_node("HAL")

        # Shared memory variables
        self.image = None
        self.v = None
        self.w = None
        self.p3d = None

        if os.getcwd() == "/":
            f = open("/RoboticsAcademy/exercises/car_junction/web-template/stop_conf.yml", "r")
        else:
            f = open("stop_conf.yml", "r")

        cfg = yaml.safe_load(f)

        ymlNode = cfg['Stop']
        #node = rospy.init_node(ymlNode["NodeName"], anonymous=True)

        # ------------ M O T O R S ----------------------------------
        # print("Publishing " + "Stop.Motors" + " with ROS messages")
        topicM = cfg['Stop']["Motors"]["Topic"]
        maxW = cfg['Stop']["Motors"]["maxW"]
        if not maxW:
            maxW = 0.5
            print("Stop.Motors"+".maxW not provided, the default value is used: " + repr(maxW))

        maxV = cfg['Stop']["Motors"]["maxV"]
        if not maxV:
            maxV = 5
            print("Stop.Motors"+".maxV not provided, the default value is used: " + repr(maxV))

        self.motors = PublisherMotors(topicM, maxV, maxW)

        # ----------------- P O S E     3 D -------------------------------------
        # print("Receiving " + "Stop.Pose3D" + " from ROS messages")
        topicP = cfg['Stop']["Pose3D"]["Topic"]
        self.pose3d = ListenerPose3d(topicP)

        # -------- C A M E R A C E N T R A L --------------------------------------
        # print("Receiving " + "Stop.CameraC" + "  CameraData from ROS messages")
        topicCameraC = cfg['Stop']["CameraC"]["Topic"]
        self.cameraC = ListenerCamera(topicCameraC)

        # -------- C A M E R A L E F T --------------------------------------------
        # print("Receiving " + "Stop.CameraL" + "  CameraData from ROS messages")
        topicCameraL = cfg['Stop']["CameraL"]["Topic"]
        self.cameraL = ListenerCamera(topicCameraL)

        # -------- C A M E R A R I G H T ------------------------------------------
        # print("Receiving " + "Stop.CameraR" + "  CameraData from ROS messages")
        topicCameraR = cfg['Stop']["CameraR"]["Topic"]
        self.cameraR = ListenerCamera(topicCameraR)

        self.template = cv2.imread('assets/img/template.png', 0)

        # Dummy Cars Controller
        self.dummy_speed_1 = -2
        self.state_msg_1 = ModelState()
        self.state_msg_1.model_name = 'car1'
        self.state_msg_1.pose.position.x = 30
        self.state_msg_1.pose.position.y = 1.5
        self.state_msg_1.pose.position.z = 0.1
        self.state_msg_1.pose.orientation.x = 0
        self.state_msg_1.pose.orientation.y = 0
        self.state_msg_1.pose.orientation.z = -0.7
        self.state_msg_1.pose.orientation.w = 0.7
        self.dummy_speed_2 = 3.5
        self.state_msg_2 = ModelState()
        self.state_msg_2.model_name = 'car2'
        self.state_msg_2.pose.position.x = -30
        self.state_msg_2.pose.position.y = -1.5
        self.state_msg_2.pose.position.z = 0.1
        self.state_msg_2.pose.orientation.x = 0
        self.state_msg_2.pose.orientation.y = 0
        self.state_msg_2.pose.orientation.z = 0.7
        self.state_msg_2.pose.orientation.w = 0.7

    # Get Image from ROS Driver Camera
    def getImage(self, lr):
        if (lr == 'left'):
            image = self.cameraL.getImage().data
        elif (lr == 'right'):
            image = self.cameraR.getImage().data
        elif (lr == 'center'):
            image = self.cameraC.getImage().data
        else:
            print("Invalid camera")

        return image

    # Set the velocity
    def setV(self, velocity):
        self.v = velocity
        self.motors.sendV(velocity)

    # Get the velocity
    def getV(self):
        velocity = self.v
        return velocity

    # Get the angular velocity
    def getW(self):
        angular = self.w
        return angular

    # Set the angular velocity
    def setW(self, angular):
        self.w = angular
        self.motors.sendW(angular)

    def getPose3D(self):
        return self.pose3d.getPose3d()

    def setPose3D(self, pose3d):
        self.pose3d = pose3d

    def getTemplate(self):
        return self.template

    def getYaw(self):
        return self.pose3d.data.yaw

    def move_dummy(self, car, time):
        # Resets dummies position
        if self.state_msg_1.pose.position.x < -70:
            self.reset_dummies()
        if self.state_msg_2.pose.position.x > 70:
            self.reset_dummies()
        if car == 1:
            self.state_msg_1.pose.position.x = self.state_msg_1.pose.position.x + time * self.dummy_speed_1
            rospy.wait_for_service('/gazebo/set_model_state')
            try:
                set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                resp = set_state(self.state_msg_1)
            except rospy.ServiceException:
                print("Service call failed")
        if car == 2:
            self.state_msg_2.pose.position.x = self.state_msg_2.pose.position.x + time * self.dummy_speed_2
            rospy.wait_for_service('/gazebo/set_model_state')
            try:
                set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                resp = set_state(self.state_msg_2)
            except rospy.ServiceException:
                print("Service call failed")
    
    def reset_dummies(self):
        self.reset_poses()
        
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state(self.state_msg_1)
        except rospy.ServiceException:
            print("Service call failed")

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state(self.state_msg_2)
        except rospy.ServiceException:
            print("Service call failed")

    def reset_poses(self):
        self.state_msg_1 = ModelState()
        self.state_msg_1.model_name = 'car1'
        self.state_msg_1.pose.position.x = 30
        self.state_msg_1.pose.position.y = 1.5
        self.state_msg_1.pose.position.z = 0.1
        self.state_msg_1.pose.orientation.x = 0
        self.state_msg_1.pose.orientation.y = 0
        self.state_msg_1.pose.orientation.z = -0.7
        self.state_msg_1.pose.orientation.w = 0.7
        self.state_msg_2 = ModelState()
        self.state_msg_2.model_name = 'car2'
        self.state_msg_2.pose.position.x = -30
        self.state_msg_2.pose.position.y = -1.5
        self.state_msg_2.pose.position.z = 0.1
        self.state_msg_2.pose.orientation.x = 0
        self.state_msg_2.pose.orientation.y = 0
        self.state_msg_2.pose.orientation.z = 0.7
        self.state_msg_2.pose.orientation.w = 0.7