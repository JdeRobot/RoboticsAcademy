#!/usr/bin/python3
#
#  Copyright (C) 1997-2019 JDE Developers Team
#
#  Authors :
#       Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
#       Francisco Miguel Rivas Montero <franciscomiguel.rivas@urjc.es>
#  Rosified by:
#       Francisco Perez Salgado <f.perez475@gmail.com>
#  Adapted to turtlebot by:
#       Julio Vega <julio.vega@urjc.es>

import rospy
from geometry_msgs.msg import Twist
import threading
from math import pi as PI
from .threadPublisher import ThreadPublisher
from copy import deepcopy


class PublisherMotors:
 
    def __init__(self, topic, maxV, maxW):
        self.maxW = maxW
        self.maxV = maxV
        self.topic = topic
        self.tw = Twist()
        self.pub = self.pub = rospy.Publisher(self.topic, Twist, queue_size=1)
        rospy.init_node("FollowLineTurtlebot")
        self.lock = threading.Lock()
        self.kill_event = threading.Event()
        self.thread = ThreadPublisher(self, self.kill_event)
        self.thread.daemon = True
        self.start()
 
    def publish (self):
        self.lock.acquire()
        tw = deepcopy(self.tw)
        self.lock.release()
        self.pub.publish(tw)
        
    def stop(self):
        self.kill_event.set()
        self.pub.unregister()

    def start (self):
        self.kill_event.clear()
        self.thread.start()

    def getMaxW(self):
        return self.maxW

    def getMaxV(self):
        return self.maxV

    def sendVelocities(self, vel):
        self.lock.acquire()
        self.tw = deepcopy(vel)
        self.lock.release()

    def sendV(self, v):
        self.sendVX(v)

    def sendL(self, l):
        self.sendVY(l)

    def sendW(self, w):
        self.sendAZ(w)

    def sendVX(self, vx):
        self.lock.acquire()
        self.tw.linear.x = vx
        self.lock.release()

    def sendVY(self, vy):
        self.lock.acquire()
        self.tw.linear.y = vy
        self.lock.release()

    def sendAZ(self, az):
        self.lock.acquire()
        self.tw.angular.z = az
        self.lock.release()

