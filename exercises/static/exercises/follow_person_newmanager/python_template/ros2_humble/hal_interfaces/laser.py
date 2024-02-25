from rclpy.node import Node
import sensor_msgs.msg
from math import pi as PI
    
### AUXILIARY FUNCTIONS
class LaserData():

    def __init__(self):

        self.values = [] # meters
        self.minAngle = 0 # Angle of first value (rads)
        self.maxAngle = 0 # Angle of last value (rads)
        self.minRange = 0 # Max Range posible (meters)
        self.maxRange = 0 #Min Range posible (meters)
        self.timeStamp = 0 # seconds

    def __str__(self):
        s = "LaserData: {\n   minAngle: " + str(self.minAngle) + "\n   maxAngle: " + str(self.maxAngle)
        s = s + "\n   minRange: " + str(self.minRange) + "\n   maxRange: " + str(self.maxRange) 
        s = s + "\n   timeStamp: " + str(self.timeStamp) + "\n   values: " + str(self.values) + "\n}"
        return s 

def laserScan2LaserData(scan):
    '''
    Translates from ROS LaserScan to JderobotTypes LaserData. 
    @param scan: ROS LaserScan to translate
    @type scan: LaserScan
    @return a LaserData translated from scan
    '''
    laser = LaserData()
    laser.values = scan.ranges
    ''' 
          ROS Angle Map      JdeRobot Angle Map
                0                  PI/2
                |                   |
                |                   |
       PI/2 --------- -PI/2  PI --------- 0
                |                   |
                |                   |
    '''
    laser.minAngle = scan.angle_min  + PI/2
    laser.maxAngle = scan.angle_max  + PI/2
    laser.maxRange = scan.range_max
    laser.minRange = scan.range_min
    laser.timeStamp = scan.header.stamp.sec + (scan.header.stamp.nanosec *1e-9)
    return laser

### HAL INTERFACE ###
class LaserNode(Node):

    def __init__(self, topic):
        super().__init__("laser_node")
        self.sub = self.create_subscription(sensor_msgs.msg.LaserScan, topic, self.listener_callback, 10)
        self.last_scan_ = sensor_msgs.msg.LaserScan()
 
    def listener_callback(self, scan):
        self.last_scan_ = scan
        
    def getLaserData(self):
        return laserScan2LaserData(self.last_scan_)