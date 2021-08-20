import sys
import config
#import comm
from PyQt5.QtWidgets import QApplication
from gui.GUI import MainWindow
from gui.threadGUI import ThreadGUI
from MyAlgorithm import MyAlgorithm

import math
import rospy
import threading
from math import asin, atan2, pi
from jderobotTypes import LaserData, CMDVel, Pose3d
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import GetModelState
from jderobotTypes import BumperData
from gui.threadPublisher import ThreadPublisher

def contactsState2BumperData(event):
    '''
    Translates from ROS ContactsState to JderobotTypes BumperData. 

    @param event: ROS ContactsState to translate

    @type event: ContactsState

    @return a BumperData translated from event

    # bumper
    LEFT   = 0
    CENTER = 1
    RIGHT  = 2

    #  state
    RELEASED = 0
    PRESSED  = 1

    '''
    bump = BumperData()
    if len(event.states) > 0:
        contact_x = event.states[0].contact_normals[0].x
        contact_y = event.states[0].contact_normals[0].y

        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        robot_coordinates = model_coordinates("roombaROS", "")
        qx = robot_coordinates.pose.orientation.x
        qy = robot_coordinates.pose.orientation.y
        qz = robot_coordinates.pose.orientation.z
        qw = robot_coordinates.pose.orientation.w

        rotateZa0=2.0*(qx*qy + qw*qz)
        rotateZa1=qw*qw + qx*qx - qy*qy - qz*qz
        robotYaw=0.0
        if(rotateZa0 != 0.0 and rotateZa1 != 0.0):
            robotYaw=math.atan2(rotateZa0,rotateZa1)

        global_contact_angle = 0.0
        if(contact_x != 0.0 and contact_y != 0.0):
            global_contact_angle = math.atan2(contact_y,contact_x)

        relative_contact_angle = global_contact_angle - robotYaw -math.pi
        
        bump.state = 1
        if relative_contact_angle <= math.pi/2 and relative_contact_angle > math.pi/6: #0.52, 1.5
            bump.bumper = 0
        elif relative_contact_angle <= math.pi/6 and relative_contact_angle > -math.pi/6: # -0.52, 0.52
            bump.bumper = 1
        elif relative_contact_angle < -math.pi/6 and relative_contact_angle >= -math.pi/2: #-1.5 , -0.52
            bump.bumper = 2
    else:
        bump.state = 0
    
    #bump.timeStamp = event.header.stamp.secs + (event.header.stamp.nsecs *1e-9)
    return bump

class ListenerBumper:
    '''
        ROS Bumper Subscriber. Bumper Client to Receive Contacts State from ROS nodes.
    '''
    def __init__(self, topic):
        '''
        ListenerBumper Constructor.

        @param topic: ROS topic to subscribe
        
        @type topic: String

        '''
        self.topic = topic
        self.data = BumperData()
        self.sub = None
        self.lock = threading.Lock()
        self.start()
 
    def __callback (self, event):
        '''
        Callback function to receive and save Contacts State. 

        @param event: ROS ContactsState received
        
        @type event: ContactsState

        '''
        bump = contactsState2BumperData(event)

        self.lock.acquire()
        self.data = bump
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
        self.sub = rospy.Subscriber(self.topic, ContactsState, self.__callback)
        
    def getBumperData(self):
        '''
        Returns last BumperData. 

        @return last JdeRobotTypes BumperData saved

        '''
        self.lock.acquire()
        bump = self.data
        self.lock.release()
        
        return bump

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
        laser.minAngle = scan.angle_min  + pi/2
        laser.maxAngle = scan.angle_max  + pi/2
        laser.maxRange = scan.range_max
        laser.minRange = scan.range_min
        laser.timeStamp = scan.header.stamp.secs + (scan.header.stamp.nsecs *1e-9)
        return laser

class ListenerLaser:

    '''
        ROS Laser Subscriber. Laser Client to Receive Laser Scans from ROS nodes.
    '''
    def __init__(self, topic):
        '''
        ListenerLaser Constructor.

        @param topic: ROS topic to subscribe
        
        @type topic: String

        '''
        self.topic = topic
        self.data = LaserData()
        self.sub = None
        self.lock = threading.Lock()
        self.start()
 
    def __callback (self, scan):
        '''
        Callback function to receive and save Laser Scans. 

        @param scan: ROS LaserScan received
        
        @type scan: LaserScan

        '''
        laser = laserScan2LaserData(scan)

        self.lock.acquire()
        self.data = laser
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
        self.sub = rospy.Subscriber(self.topic, LaserScan, self.__callback)
        
    def getLaserData(self):
        '''
        Returns last LaserData. 

        @return last JdeRobotTypes LaserData saved

        '''
        self.lock.acquire()
        laser = self.data
        self.lock.release()
        
        return laser

def cmdvel2Twist(vel):
    '''
    Translates from JderobotTypes CMDVel to ROS Twist. 

    @param vel: JderobotTypes CMDVel to translate

    @type img: JdeRobotTypes.CMDVel

    @return a Twist translated from vel

    '''
    tw = Twist()
    tw.linear.x = vel.vx
    tw.linear.y = vel.vy
    tw.linear.z = vel.vz
    tw.angular.x = vel.ax
    tw.angular.y = vel.ay
    tw.angular.z = vel.az

    return tw

class PublisherMotors:
    '''
        ROS Motors Publisher. Motors Client to Send CMDVel to ROS nodes.
    '''
    def __init__(self, topic, maxV, maxW):
        '''
        ListenerMotors Constructor.

        @param topic: ROS topic to publish
        
        @type topic: String

        '''
        self.maxW = maxW
        self.maxV = maxV

        self.topic = topic
        self.data = CMDVel()
        self.pub = self.pub = rospy.Publisher(self.topic, Twist, queue_size=1)
        self.lock = threading.Lock()

        self.kill_event = threading.Event()
        self.thread = ThreadPublisher(self, self.kill_event)

        self.thread.daemon = True
        self.start()
 
    def publish (self):
        '''
        Function to publish cmdvel. 
        '''
        self.lock.acquire()
        tw = cmdvel2Twist(self.data)
        self.lock.release()
        self.pub.publish(tw)
        
    def stop(self):
        '''
        Stops (Unregisters) the client. If client is stopped you can not start again, Threading.Thread raised error

        '''
        self.kill_event.set()
        self.pub.unregister()

    def start (self):
        '''
        Starts (Subscribes) the client. If client is stopped you can not start again, Threading.Thread raised error

        '''
        self.kill_event.clear()
        self.thread.start()
        


    def getMaxW(self):
        return self.maxW

    def getMaxV(self):
        return self.maxV
        

    def sendVelocities(self, vel):
        '''
        Sends CMDVel.

        @param vel: CMDVel to publish
        
        @type vel: CMDVel

        '''
        self.lock.acquire()
        self.data = vel
        self.lock.release()

    def sendV(self, v):
        '''
        Sends V velocity. uses self.sendVX

        @param v: V velocity
        
        @type v: float

        '''
        self.sendVX(v)

    def sendL(self, l):
        '''
        Sends L velocity. uses self.sendVY

        @param l: L velocity
        
        @type l: float

        '''
        self.sendVY(l)

    def sendW(self, w):
        '''
        Sends W velocity. uses self.sendAZ

        @param w: W velocity
        
        @type w: float

        '''
        self.sendAZ(w)

    def sendVX(self, vx):
        '''
        Sends VX velocity.

        @param vx: VX velocity
        
        @type vx: float

        '''
        self.lock.acquire()
        self.data.vx = vx
        self.lock.release()

    def sendVY(self, vy):
        '''
        Sends VY velocity.

        @param vy: VY velocity
        
        @type vy: float

        '''
        self.lock.acquire()
        self.data.vy = vy
        self.lock.release()

    def sendAZ(self, az):
        '''
        Sends AZ velocity.

        @param az: AZ velocity
        
        @type az: float

        '''
        self.lock.acquire()
        self.data.az = az
        self.lock.release()

def quat2Yaw(qw, qx, qy, qz):
    '''
    Translates from Quaternion to Yaw. 

    @param qw,qx,qy,qz: Quaternion values

    @type qw,qx,qy,qz: float

    @return Yaw value translated from Quaternion

    '''
    rotateZa0=2.0*(qx*qy + qw*qz)
    rotateZa1=qw*qw + qx*qx - qy*qy - qz*qz
    rotateZ=0.0
    if(rotateZa0 != 0.0 and rotateZa1 != 0.0):
        rotateZ=atan2(rotateZa0,rotateZa1)
    return rotateZ

def quat2Pitch(qw, qx, qy, qz):
    '''
    Translates from Quaternion to Pitch. 

    @param qw,qx,qy,qz: Quaternion values

    @type qw,qx,qy,qz: float

    @return Pitch value translated from Quaternion

    '''

    rotateYa0=-2.0*(qx*qz - qw*qy)
    rotateY=0.0
    if(rotateYa0 >= 1.0):
        rotateY = pi/2.0
    elif(rotateYa0 <= -1.0):
        rotateY = -pi/2.0
    else:
        rotateY = asin(rotateYa0)

    return rotateY

def quat2Roll (qw, qx, qy, qz):
    '''
    Translates from Quaternion to Roll. 

    @param qw,qx,qy,qz: Quaternion values

    @type qw,qx,qy,qz: float

    @return Roll value translated from Quaternion

    '''
    rotateXa0=2.0*(qy*qz + qw*qx)
    rotateXa1=qw*qw - qx*qx - qy*qy + qz*qz
    rotateX=0.0

    if(rotateXa0 != 0.0 and rotateXa1 != 0.0):
        rotateX=atan2(rotateXa0, rotateXa1)
    return rotateX


def odometry2Pose3D(odom):
    '''
    Translates from ROS Odometry to JderobotTypes Pose3d. 

    @param odom: ROS Odometry to translate

    @type odom: Odometry

    @return a Pose3d translated from odom

    '''
    pose = Pose3d()
    ori = odom.pose.pose.orientation

    pose.x = odom.pose.pose.position.x
    pose.y = odom.pose.pose.position.y
    pose.z = odom.pose.pose.position.z
    #pose.h = odom.pose.pose.position.h
    pose.yaw = quat2Yaw(ori.w, ori.x, ori.y, ori.z)
    pose.pitch = quat2Pitch(ori.w, ori.x, ori.y, ori.z)
    pose.roll = quat2Roll(ori.w, ori.x, ori.y, ori.z)
    pose.q = [ori.w, ori.x, ori.y, ori.z]
    pose.timeStamp = odom.header.stamp.secs + (odom.header.stamp.nsecs *1e-9)

    return pose


class ListenerPose3d:
    '''
        ROS Pose3D Subscriber. Pose3D Client to Receive pose3d from ROS nodes.
    '''
    def __init__(self, topic):
        '''
        ListenerPose3d Constructor.

        @param topic: ROS topic to subscribe
        
        @type topic: String

        '''
        self.topic = topic
        self.data = Pose3d()
        self.sub = None
        self.lock = threading.Lock()
        self.start()
 
    def __callback (self, odom):
        '''
        Callback function to receive and save Pose3d. 

        @param odom: ROS Odometry received
        
        @type odom: Odometry

        '''
        pose = odometry2Pose3D(odom)

        self.lock.acquire()
        self.data = pose
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
        self.sub = rospy.Subscriber(self.topic, Odometry, self.__callback)
        
    def getPose3d(self):
        '''
        Returns last Pose3d. 

        @return last JdeRobotTypes Pose3d saved

        '''
        self.lock.acquire()
        pose = self.data
        self.lock.release()
        
        return pose


if __name__ == "__main__":

    cfg = config.load(sys.argv[1])
    ymlNode = cfg.getProperty('VacuumCleaner')
    node = rospy.init_node(ymlNode["NodeName"], anonymous=True)

    # ------------ M O T O R S ----------------------------------
    print("Publishing "+  "VacuumCleaner.Motors" + " with ROS messages")
    topicM = cfg.getProperty("VacuumCleaner.Motors"+".Topic")
    maxW = cfg.getPropertyWithDefault("VacuumCleaner.Motors"+".maxW", 0.5)
    if not maxW:
        maxW = 0.5
        print ("VacuumCleaner.Motors"+".maxW not provided, the default value is used: "+ repr(maxW))
    maxV = cfg.getPropertyWithDefault("VacuumCleaner.Motors"+".maxV", 5)
    if not maxV:
        maxV = 5
        print ("VacuumCleaner.Motors"+".maxV not provided, the default value is used: "+ repr(maxV))
    motors = PublisherMotors(topicM, maxV, maxW)
    # ----------------- P O S E     3 D -------------------------------------
    print("Receiving " + "VacuumCleaner.Pose3D" + " from ROS messages")
    topicP = cfg.getProperty("VacuumCleaner.Pose3D"+".Topic")
    pose3d = ListenerPose3d(topicP)
    # -------- L A S E R --------------------------------------------------
    print("Receiving " + "VacuumCleaner.Laser" + "  LaserData from ROS messages")
    topicL  = cfg.getProperty("VacuumCleaner.Laser"+".Topic")
    laser = ListenerLaser(topicL)
    # -------- B U M P E R --------------------------------------------------
    print("Receiving " + "VacuumCleaner.Bumper" + " from ROS messages")
    topicB = cfg.getProperty("VacuumCleaner.Bumper"+".Topic")
    bumper = ListenerBumper(topicB)

    algorithm=MyAlgorithm(pose3d, motors,laser, bumper)

    app = QApplication(sys.argv)
    myGUI = MainWindow(pose3d)
    myGUI.setMotors(motors)
    myGUI.setPose3D(pose3d)
    myGUI.setLaser(laser)
    myGUI.setBumper(bumper)
    myGUI.setAlgorithm(algorithm)
    myGUI.show()


    t2 = ThreadGUI(myGUI)
    t2.daemon=True
    t2.start()


    sys.exit(app.exec_())
