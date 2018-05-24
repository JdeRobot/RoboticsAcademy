import sys, traceback
import jderobot


class Beacon:
    def __init__(self,id,pose,active=False,reached=False):
        self.id=id
        self.pose=pose
        self.active=active
        self.reached=reached

    def getPose(self):
        return self.pose

    def getId(self):
        return self.id

    def getPose(self):
        return self.pose

    def isReached(self):
        return self.reached

    def setReached(self,value):
        self.reached=value

    def isActive(self):
        return self.active

    def setActive(self,value):
        self.active=value

