import rospy
from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d
from interfaces.laser import ListenerLaser

# Hardware Abstraction Layer
class HAL:
    IMG_WIDTH = 320
    IMG_HEIGHT = 240

    # Initialize components at the module level
    rospy.init_node("HAL")
    motors = PublisherMotors("/taxi_holo/cmd_vel", 4, 0.3)
    pose3d = ListenerPose3d("/taxi_holo/odom")
    laser_front = ListenerLaser("/F1ROS/laser_f/scan")
    laser_right = ListenerLaser("/F1ROS/laser_r/scan")
    laser_back = ListenerLaser("/F1ROS/laser_b/scan")

    @staticmethod
    def setV(velocity):
        HAL.motors.sendV(velocity)

    @staticmethod
    def setW(velocity):
        HAL.motors.sendW(velocity)

    @staticmethod
    def getPose3d():
        return HAL.pose3d.getPose3d()

    @staticmethod
    def getFrontLaserData():
        return HAL.laser_front.getLaserData()

    @staticmethod
    def getRightLaserData():
        return HAL.laser_right.getLaserData()

    @staticmethod
    def getBackLaserData():
        return HAL.laser_back.getLaserData()
