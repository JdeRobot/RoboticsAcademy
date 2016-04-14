from sensors import sensor
import numpy as np
import threading
import jderobot
import math
from Target import Target
from Parser import Parser

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

        # Init targets
        parser = Parser('targets.json')
        self.targets = parser.getTargets()

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

