import rospy
from nav_msgs.msg import Path
import threading
from math import asin, atan2, pi

class ListenerPath:
    '''
        ROS Path Subscriber. Path Client to Receive path from ROS nodes.
    '''
    def __init__(self, topic):
        '''
        ListenerPath Constructor.

        @param topic: ROS topic to subscribe
        
        @type topic: String

        '''
        self.topic = topic
        self.data = []
        self.sub = None
        self.lock = threading.Lock()
        self.start()
 
    def __callback (self, path):
        '''
        Callback function to receive and save Path. 

        @param path: ROS Path received
        
        @type path: Path

        '''
        self.lock.acquire()
        self.data = path.poses
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
        # rospy.init_node('listener', anonymous=True)
        self.sub = rospy.Subscriber(self.topic, Path, self.__callback)
        # rospy.spin()
        
    def getPath(self):
        '''
        Returns last Path. 

        @return last poses array saved

        '''
        self.lock.acquire()
        path = self.data
        self.lock.release()
        
        return path