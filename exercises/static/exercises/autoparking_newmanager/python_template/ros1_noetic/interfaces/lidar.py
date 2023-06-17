import rospy
from sensor_msgs.msg import PointCloud
import threading
from math import pi as PI

class LidarData ():

    def __init__(self):

        self.values = [] # meters
        self.minAngle = 0 # Angle of first value (rads)
        self.maxAngle = 0 # Angle of last value (rads)
        self.minRange = 0 # Max Range posible (meters)
        self.maxRange = 0 #Min Range posible (meters)
        self.timeStamp = 0 # seconds


    def __str__(self):
        s = "LidarData: {\n   minAngle: " + str(self.minAngle) + "\n   maxAngle: " + str(self.maxAngle)
        s = s + "\n   minRange: " + str(self.minRange) + "\n   maxRange: " + str(self.maxRange) 
        s = s + "\n   timeStamp: " + str(self.timeStamp) + "\n   values: " + str(self.values) + "\n}"
        return s 


def LidarScan2LidarData(scan):
    '''
    Translates from ROS LidarScan to JderobotTypes LidarData. 

    @param scan: ROS LidarScan to translate

    @type scan: LidarScan

    @return a LidarData translated from scan

    '''
    Lidar = LidarData()
    Lidar.values = scan.channels
    ''' 
          ROS Angle Map      JdeRobot Angle Map
                0                  PI/2
                |                   |
                |                   |
       PI/2 --------- -PI/2  PI --------- 0
                |                   |
                |                   |
    '''
    Lidar.minAngle = 0
    Lidar.maxAngle = 2*PI
    Lidar.maxRange = 0
    Lidar.minRange = 10
    Lidar.timeStamp = scan.header.stamp.secs + (scan.header.stamp.nsecs *1e-9)
    return Lidar

class ListenerLidar:
    '''
        ROS Lidar Subscriber. Lidar Client to Receive Lidar Scans from ROS nodes.
    '''
    def __init__(self, topic):
        '''
        ListenerLidar Constructor.

        @param topic: ROS topic to subscribe
        
        @type topic: String

        '''
        self.topic = topic
        self.data = LidarData()
        self.sub = None
        self.lock = threading.Lock()
        self.start()
 
    def __callback (self, scan):
        '''
        Callback function to receive and save Lidar Scans. 

        @param scan: ROS LidarScan received
        
        @type scan: LidarScan

        '''
        #Lidar = LidarScan2LidarData(scan)

        self.lock.acquire()
        self.data = scan
        self.lock.release()
        
    def stop(self):
        '''
        Stops (Unregisters) the client.

        '''
        self.sub.unregister()

    def start (self):
        '''
        Starts (Subscribes) the client.

        '''
        self.sub = rospy.Subscriber(self.topic, PointCloud, self.__callback)
        
    def getLidarData(self):
        '''
        Returns last LidarData. 

        @return last JdeRobotTypes LidarData saved

        '''
        self.lock.acquire()
        Lidar = self.data
        self.lock.release()
        
        return Lidar
