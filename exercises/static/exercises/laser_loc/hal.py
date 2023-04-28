import rospy
import cv2
import threading
import time
from datetime import datetime

from parse_configuration import Config

from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d
from interfaces.laser import ListenerLaser
from interfaces.sonar import ListenerSonar

# Hardware Abstraction Layer
class HAL:
    
    def __init__(self):
        self.config = Config()
        self.motors = PublisherMotors(self.config.topic_motors, self.config.max_velV, self.config.max_velW)
        self.pose3d = ListenerPose3d(self.config.topic_pose)
        self.sonar_0 = ListenerSonar(self.config.topic_sonar_0)
        self.sonar_1 = ListenerSonar(self.config.topic_sonar_1)
        self.sonar_2 = ListenerSonar(self.config.topic_sonar_2)
        self.sonar_3 = ListenerSonar(self.config.topic_sonar_3)
        self.sonar_4 = ListenerSonar(self.config.topic_sonar_4)
        self.sonar_5 = ListenerSonar(self.config.topic_sonar_5)
        self.sonar_6 = ListenerSonar(self.config.topic_sonar_6)
        self.sonar_7 = ListenerSonar(self.config.topic_sonar_7)
        self.laser = ListenerLaser(self.config.topic_laser)

    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initRobot(cls):
        new_instance = cls()
        return new_instance

    def setV(self, velocity):
        self.motors.sendV(velocity)

    def setW(self, velocity):
        self.motors.sendW(velocity)
    
    def getPose3d(self):
        return self.pose3d.getPose3d()

    def getLaserData(self):
        return self.laser.getLaserData()
    
    def getSonarData_0(self):
        return self.sonar_0.getSonarData()

    def getSonarData_1(self):
        return self.sonar_1.getSonarData()

    def getSonarData_2(self):
        return self.sonar_2.getSonarData()

    def getSonarData_3(self):
        return self.sonar_3.getSonarData()

    def getSonarData_4(self):
        return self.sonar_4.getSonarData()

    def getSonarData_5(self):
        return self.sonar_5.getSonarData()

    def getSonarData_6(self):
        return self.sonar_6.getSonarData()

    def getSonarData_7(self):
        return self.sonar_7.getSonarData()
