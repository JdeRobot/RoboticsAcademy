import rospy
from mavros_msgs.srv import CommandBool,CommandTOL, SetMode 
import threading
import time

class PublisherExtra:
    '''
        ROS CMDVel Publisher. CMDVel Client to Send CMDVel to ROS nodes.
    '''
    def __init__(self, topicArming, topicLand, topicSetMode):
        '''
        PublisherCMDVel Constructor.

        @param topic: ROS topic to publish
        
        @type topic: String

        '''
        self.topicArming = topicArming
        self.topicLand = topicLand
        self.topicSetMode = topicSetMode

        self.lock = threading.Lock()


        self.arming_client = rospy.ServiceProxy(topicArming,CommandBool)
        self.land_client = rospy.ServiceProxy(topicLand,CommandTOL)        
        self.set_mode_client = rospy.ServiceProxy(topicSetMode,SetMode)

        

    def arming(self):
        self.lock.acquire()
        self.arming_client.call(value=True)
        time.sleep(0.5)
        self.set_mode_client.call(custom_mode="OFFBOARD")
        self.lock.release()
        
    def land(self):
        self.lock.acquire()
        self.land_client.call(0,0,0,0,0)
        self.lock.release()
        
    def toggleCam(self):
        pass
              
    def reset(self):
        pass
        
    def record(self,record):
        pass


