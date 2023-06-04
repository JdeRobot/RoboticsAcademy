import rospy
import cv2
from interfaces.motors import PublisherMotors


# Hardware Abstraction Layer
class HAL:

    def __init__(self):
        rospy.init_node("HAL")

        self.cameraCapture = cv2.VideoCapture(0)
        self.motors = PublisherMotors("/F1ROS/cmd_vel", 4, 0.3)

    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initRobot(self):
        pass

    def setV(self, velocity):
        self.motors.sendV(velocity)
    
    def setW(self, velocity):
        self.motors.sendW(velocity)

    # Get Image from ROS Driver Camera
    def getImage(self):
        success, frame = self.cameraCapture.read()
        return frame
