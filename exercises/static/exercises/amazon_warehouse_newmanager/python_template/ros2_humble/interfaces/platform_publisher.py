import rclpy
from std_msgs.msg import String
import threading
from math import pi as PI
from .threadPublisher import ThreadPublisher

def cmdLift2String(cmdLift):
    return cmdLift.msg

class CMDLift ():

    def __init__(self):
        self.msg = String()
        self.msg.data = "unload"
    
    def cmd(self,cmd):
        self.msg.data = cmd

    def __str__(self):
        return "CMDlift:" + self.msg.data

class PublisherPlatform:
 
    def __init__(self, topic):
        self.node = rclpy.create_node('PublisherPlatform')
        self.topic = topic
        self.data = CMDLift()
        self.pub = self.node.create_publisher(String, self.topic, 10)
        
        self.lock = threading.Lock()
        self.kill_event = threading.Event()
        self.thread = ThreadPublisher(self, self.kill_event)

        self.thread.daemon = True
        self.start()
 
    def publish(self):
        self.lock.acquire()
        msg = cmdLift2String(self.data)
        self.pub.publish(msg)
        self.lock.release()
        
    def stop(self):
        self.kill_event.set()
        self.pub.unregister()

    def start (self):
        self.kill_event.clear()
        self.thread.start()
        
    def load(self):
        self.lock.acquire()
        self.data.cmd("load")
        self.lock.release()

    def unload(self):
        self.lock.acquire()
        self.data.cmd("unload")
        self.lock.release()



