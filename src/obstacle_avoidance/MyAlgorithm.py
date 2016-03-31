from sensors import sensor
import numpy as np
import threading
import jderobot
import math
from Target import Target

class MyAlgorithm():

    def __init__(self, sensor):
        self.sensor = sensor
        self.imageRight=None
        self.imageLeft=None
        self.lock = threading.Lock()

        # Car direction
        self.carx = 0.0
        self.cary = 0.0

        # Obstacles direction
        self.obsx = 0.0
        self.obsy = 0.0

        # Average direction
        self.avgx = 0.0
        self.avgy = 0.0

        # Current target
        self.targetx = 0.0
        self.targety = 0.0

        self.initTargets()

    def initTargets(self):
        self.targets = []
        self.targets.append(Target('target01',jderobot.Pose3DData(1,-30,0,0,0,0,0,0),False,False))
        self.targets.append(Target('target02',jderobot.Pose3DData(-5,-41,0,0,0,0,0,0),False,False))
        self.targets.append(Target('target03',jderobot.Pose3DData(-12,-33,0,0,0,0,0,0),False,False))
        self.targets.append(Target('target04',jderobot.Pose3DData(-15,-14,0,0,0,0,0,0),False,False))
        self.targets.append(Target('target05',jderobot.Pose3DData(-54,-13,0,0,0,0,0,0),False,False))
        self.targets.append(Target('target06',jderobot.Pose3DData(-67,-29,0,0,0,0,0,0),False,False))
        self.targets.append(Target('target07',jderobot.Pose3DData(-71,3,0,0,0,0,0,0),False,False))
        self.targets.append(Target('target08',jderobot.Pose3DData(-49,6,0,0,0,0,0,0),False,False))
        self.targets.append(Target('target09',jderobot.Pose3DData(-49,20,0,0,0,0,0,0),False,False))
        self.targets.append(Target('target10',jderobot.Pose3DData(-118,20,0,0,0,0,0,0),False,False))
        self.targets.append(Target('target11',jderobot.Pose3DData(-116,8,0,0,0,0,0,0),False,False))
        self.targets.append(Target('target12',jderobot.Pose3DData(-106,-2,0,0,0,0,0,0),False,False))
        self.targets.append(Target('target13',jderobot.Pose3DData(-150,-4,0,0,0,0,0,0),False,False))
        self.targets.append(Target('target14',jderobot.Pose3DData(-150,40,0,0,0,0,0,0),False,False))
        self.targets.append(Target('target15',jderobot.Pose3DData(-106,41,0,0,0,0,0,0),False,False))
        self.targets.append(Target('target16',jderobot.Pose3DData(-96,29,0,0,0,0,0,0),False,False))
        self.targets.append(Target('target17',jderobot.Pose3DData(-84,43,0,0,0,0,0,0),False,False))
        self.targets.append(Target('target18',jderobot.Pose3DData(-46,49,0,0,0,0,0,0),False,False))
        self.targets.append(Target('target19',jderobot.Pose3DData(-40,62,0,0,0,0,0,0),False,False))
        self.targets.append(Target('target20',jderobot.Pose3DData(-31,45,0,0,0,0,0,0),False,False))
        self.targets.append(Target('target21',jderobot.Pose3DData(-20,45,0,0,0,0,0,0),False,False))
        self.targets.append(Target('target22',jderobot.Pose3DData(-17,57,0,0,0,0,0,0),False,False))
        self.targets.append(Target('target23',jderobot.Pose3DData(-1,57,0,0,0,0,0,0),False,False))
        self.targets.append(Target('target24',jderobot.Pose3DData(0,0,0,0,0,0,0,0),False,False))

    def getNextTarget(self):
        for target in self.targets:
            if target.isReached() == False:
                return target

        return None

    def execute(self):
        self.currentTarget=self.getNextTarget()
        self.targetx = self.currentTarget.getPose().x
        self.targety = self.currentTarget.getPose().y

        # TODO

    # Gui functions
    def setRightImageFiltered(self, image):
        self.lock.acquire()
        self.imageRight=image
        self.lock.release()

    def setLeftImageFiltered(self, image):
        self.lock.acquire()
        self.imageLeft=image
        self.lock.release()

    def getRightImageFiltered(self):
        self.lock.acquire()
        tempImage=self.imageRight
        self.lock.release()
        return tempImage

    def getLeftImageFiltered(self):
        self.lock.acquire()
        tempImage=self.imageLeft
        self.lock.release()
        return tempImage

    def getCarDirection(self):
        return (self.carx, self.cary)

    def getObstaclesDirection(self):
        return (self.obsx, self.obsy)

    def getAverageDirection(self):
        return (self.avgx, self.avgy)

    def getCurrentTarget(self):
        return (self.targetx, self.targety)







        

