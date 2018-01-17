#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import threading
import time
from datetime import datetime
import jderobot
import math
import cv2
from math import pi as pi
import random

time_cycle = 80

class MyAlgorithm(threading.Thread):

    def __init__(self, pose3d, motors, laser, bumper):
        self.pose3d = pose3d
        self.motors = motors
        self.laser = laser
        self.bumper = bumper

        self.map = cv2.imread("resources/images/mapgrannyannie.png", cv2.IMREAD_GRAYSCALE)
        self.map = cv2.resize(self.map, (500, 500))

        self.grid = np.ones([500, 500], float)

        self.orientation = 'left'

        self.secondTurn = True
        self.horizontal = True
        self.crash = False
        self.firstTurn = False
        self.crashObstacle = False
        self.saturation = False
        self.obstacleRight = False
        self.noObstRight = False
        self.corner = False
        self.sizeVacuum = False

        self.startTime = 0
        self.time = 0
        self.timeSat = 0
        self.yaw = 0

        self.DIST_TO_OBST_RIGHT = 30
        self.DIST_MIN_TO_OBST_RIGHT = 15
        self.DIST_TO_OBST_FRONT = 15
        self.MARGIN = 0.2
        self.MARGIN_OBST_RIGHT = 0.1
        self.TIME_PERIM = 60
        self.NUM_ROWS_COLUMNS = 10
        self.SCALE = 50
        self.MAX_VAL_GRID = 100
        self.ADD_VAL_GRID = 10
        self.SUB_VAL_GRID = 2
        self.MAX_SQUARES = 3
        self.SECONDS_REDUCE = 1
        self.SECONDS_SAT = 200
        self.VACUUM_SIZE = 9
        self.ADD_VAL_MAP = 128

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)


    def parse_laser_data(self,laser_data):
        laser = []
        for i in range(laser_data.numLaser):
            dist = laser_data.distanceData[i]/1000.0
            angle = math.radians(i)
            laser += [(dist, angle)]
        return laser


    def laser_vector(self,laser_array):
        laser_vectorized = []
        for d,a in laser_array:
            x = d * math.cos(a) * -1
            y = d * math.sin(a) * -1
            v = (x, y)
            laser_vectorized += [v]
        return laser_vectorized


    def run (self):
        while (not self.kill_event.is_set()):

            start_time = datetime.now()

            if not self.stop_event.is_set():
                self.execute()

            finish_Time = datetime.now()


            dt = finish_Time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            #print (ms)
            if (ms < time_cycle):
                time.sleep((time_cycle - ms) / 1000.0)


    def stop (self):
        self.stop_event.set()


    def play (self):
        if self.is_alive():
            self.stop_event.clear()
        else:
            self.start()


    def kill (self):
        self.kill_event.set()


    ######    CRASH FUNCTIONS   ######

    def checkBumper(self):
        for i in range(0, 350):
            # Returns 1 if it collides, and 0 if it doesn't collides
            crash = self.bumper.getBumperData().state
            if crash == 1:
                self.stopVacuum()
                break
        return crash


    def checkLaser(self):
        crash = 0
        # Get the data of the laser sensor, which consists of 180 pairs of values
        laser_data = self.laser.getLaserData()

        # Distance in millimeters, we change to cm
        laserCenter = laser_data.distanceData[90]/10
        if laserCenter <= 10:
            crash = 1
            self.stopVacuum
        return crash


    def checkCrash(self):
        # Bumper
        crashBumper = self.checkBumper()
        # Laser
        crashLaser= self.checkLaser()

        if crashBumper == 1 or crashLaser == 1:
            crash = 1
        else:
            crash = 0

        return crash


    ######   SATURATION FUNCTIONS   ######

    def initSatTime(self):
        if self.time == 0:
            self.time = time.time()
        if self.timeSat == 0:
            self.timeSat = time.time()


    def reduceValueSquare(self, numRow, numColumn):
        # Reduce the value of a particular square
        for i in range((numColumn * self.SCALE), (numColumn * self.SCALE + self.SCALE)):
            for j in range((numRow * self.SCALE), (numRow * self.SCALE + self.SCALE)):
                if self.grid[i][j] != 0:
                    self.grid[i][j] = self.grid[i][j] - self.SUB_VAL_GRID


    def reduceValueGrid(self):
        # Scrolls the entire image
        for i in range(0, self.NUM_ROWS_COLUMNS):
            for j in range(0, self.NUM_ROWS_COLUMNS):
                self.reduceValueSquare(i, j)


    def changeValuesGrid(self):
        # Change the value of the grid depending on where the vacuum goes
        x = self.pose3d.getX()
        y = self.pose3d.getY()

        final_poses = self.RTVacuum() * np.matrix([[x], [y], [1], [1]]) * self.SCALE

        # Grid 500 x 500 and we want a grid of 10 x 10
        # We keep the whole section of the division to know in which square this is
        numX = int(final_poses.flat[0] / self.SCALE)
        numY = int(final_poses.flat[1] / self.SCALE)

        for i in range((numX * self.SCALE), (numX * self.SCALE + self.SCALE)):
            for j in range((numY * self.SCALE), (numY * self.SCALE + self.SCALE)):
                if self.grid[j][i] < self.MAX_VAL_GRID:
                    self.grid[j][i] = self.grid[j][i] + self.ADD_VAL_GRID


    def showGrid(self):
        # To show the grid well
        # Maximum value of the pixels
		maxVal = np.amax(self.grid)
		if maxVal != 0:
		    # Saves a copy of the image but divided by the maximum value so that it is not saturated
			copy = np.dot(self.grid, (1/maxVal))
		else:
			 copy = self.grid
		cv2.imshow("Grid ", copy)


    def checkSaturation(self):
        saturation = False
        numSquaresVisited = 0
        for i in range(0, self.NUM_ROWS_COLUMNS):
            for j in range(0, self.NUM_ROWS_COLUMNS):
                # Check the first pixel of the square because they all have the same value
                valuePos = self.grid[j*self.SCALE][i*self.SCALE]
                if valuePos != 0:
                    numSquaresVisited = numSquaresVisited + 1

        if numSquaresVisited < self.MAX_SQUARES:
            saturation = True
        return saturation


    def checkSaturationVacuum(self):
        timeNow = time.time()
        if self.saturation == False:
            if abs(self.time - timeNow) >= self.SECONDS_REDUCE:
                # If 5 seconds have elapsed we reduce the value of the squares of the grid
                self.reduceValueGrid()
                self.time = 0

            if abs(self.timeSat - timeNow) >= self.SECONDS_SAT:
                self.saturation = self.checkSaturation()
                if self.saturation == True:
                    # Stop
                    self.stopVacuum()
                self.timeSat = 0


    ######   VACUUM FUNCTIONS   #######

    def RTy(self, angle, tx, ty, tz):
        RT = np.matrix([[math.cos(angle), 0, math.sin(angle), tx], [0, 1, 0, ty], [-math.sin(angle), 0, math.cos(angle), tz], [0,0,0,1]])
        return RT


    def RTVacuum(self):
        RTy = self.RTy(pi, 5.8, 4, 0)
        return RTy


    def stopVacuum(self):
        self.motors.sendW(0)
        self.motors.sendV(0)


    def stopAndBackwards(self):
        # Stop
        self.stopVacuum()
        time.sleep(1)
        # Go backwards
        self.motors.sendV(-0.1)
        time.sleep(1)


    def goForward(self,v):
        self.motors.sendW(0)
        self.motors.sendV(v)


    def returnOrientation(self, yaw):
        if -pi/2 <= yaw <= pi/2:
            orientation = 'left'
        elif pi/2 <= yaw <= pi or -pi <= yaw <= -pi/2:
            orientation = 'right'
        return orientation


    def turn90(self, angle1, angle2, yawNow):
        # angle1: orientacion a la que tienes que llegar si la orientacion es izq
        # angle2: orientacion a la que tienes que llegar si la orientacion es decha
        # yawNow: orientacion actual
        turning = True
        rangeDegrees = 0.125

        if angle1 == pi or angle2 == 0:
            rangeDegrees = 0.145

        if angle2 == pi and yawNow < 0:
            angle2 = -angle2

        if (self.orientation == 'left') and (yawNow <= (angle1-rangeDegrees) or yawNow >= (angle1+rangeDegrees)):
            # Look left and turn to left
            self.motors.sendV(0)
            self.motors.sendW(0.2)
        elif (self.orientation == 'right') and (yawNow <= (angle2-rangeDegrees) or yawNow >= (angle2+rangeDegrees)):
            # Look right and turn to right
            self.motors.sendV(0)
            self.motors.sendW(-0.2)
        else:
            turning = False
            self.motors.sendW(0)
        return turning


    def restartVariables(self):
        self.startTime = 0
        self.crashObstacle = False
        self.obstacleRight = False
        self.noObstRight = False
        self.corner = False
        self.sizeVacuum = False
        self.saturation = False



    ######   MAP FUNCTIONS   ######

    def whitePixels(self):
        # Calculate the white pixels of the map
        numPixels = 0
        for i in range(0, self.map.shape[1]):
            for j in range(0, self.map.shape[0]):
                if self.map[i][j] == 255:
                    numPixels = numPixels + 1
        return numPixels


    def changeMap(self):
        # Change the value of the pixels depending on where the vacuum goes
        x = self.pose3d.getPose3d().x
        y = self.pose3d.getPose3d().y

        final_poses = self.RTVacuum() * np.matrix([[x], [y], [1], [1]]) * self.SCALE

        poseX = int(final_poses.flat[0])
        poseY = int(final_poses.flat[1])

        #self.map[poseY][poseX] = self.ADD_VAL_MAP

        for i in range((poseY - self.VACUUM_SIZE), (poseY + self.VACUUM_SIZE)):
            for j in range((poseX - self.VACUUM_SIZE), (poseX + self.VACUUM_SIZE)):
                self.map[i][j] = self.ADD_VAL_MAP
        self.map[poseY][poseX] = 0
        cv2.imshow("MAP ", self.map)


    ######   PERIMETER FUNCTIONS   ######

    def initPerimTime(self):
        if self.startTime == 0:
            self.startTime = time.time()


    def calculateSideTriangle(self, a, b, angle):
        c = math.sqrt(pow(a,2) + pow(b,2) - 2*a*b*math.cos(angle))
        return c


    def calculateAngleTriangle(self, a, b, c):
        numer = pow(a,2) + pow(b,2) - pow(c,2)
        deno = 2 * a * b
        angleC = math.acos(numer/deno)
        return angleC


    def turnUntilObstacleToRight(self, angle):
        # Turn until the obstacle is to the right
        if (angle >= pi/2 + self.MARGIN_OBST_RIGHT or angle <= pi/2 - self.MARGIN_OBST_RIGHT) and self.obstacleRight == False:
            # Turn to left
            self.motors.sendV(0)
            self.motors.sendW(0.2)
        else:
            self.obstacleRight = True
            self.motors.sendW(0)


    def turnCorner(self, yaw):
        self.corner = True
        # Stop
        self.motors.sendV(0)

        if self.yaw <= (pi + self.MARGIN) and self.yaw >= (pi - self.MARGIN):
            self.yaw = -pi

        # Gira 90 grados a la izq
        self.orientation = 'left'
        giro = self.turn90(self.yaw + pi/2, pi/2, yaw)
        if giro == False:
            self.motors.sendW(0)
            self.corner = False


    def goNearToWall(self, laserRight):
        if laserRight <= self.DIST_MIN_TO_OBST_RIGHT:
            # Turn to left
            self.motors.sendV(0)
            self.motors.sendW(0.1)
        elif laserRight >= self.DIST_TO_OBST_RIGHT:
            # Turn to right
            self.motors.sendV(0)
            self.motors.sendW(-0.1)
        else:
            self.motors.sendW(0)
            self.motors.sendV(0.1)


    def execute(self):

        # TODO
        '''
        # Time to check saturation
        self.initSatTime()

        # Check saturation
        self.checkSaturationVacuum()

        # Change and show grid
        self.changeValuesGrid()
        self.showGrid()
        '''
        # Vacuum's poses
        x = self.pose3d.getPose3d().x
        y = self.pose3d.getPose3d().y
        yaw = self.pose3d.getPose3d().yaw

        # Check crash
        crash = self.checkCrash()

        self.changeMap()

        if self.saturation == False:
            if crash == 1 and self.crash == False:
                print ("CRAAASH")
                # Stop and go backwards
                self.stopAndBackwards()

                # Yaw
                yaw = self.pose3d.getPose3d().yaw
                self.firstTurn = False
                self.crash = True

            if self.firstTurn == False and self.crash == True:
                print ("First turn")
                # Yaw
                yawNow = self.pose3d.getPose3d().yaw
                # Orientation
                self.orientation = self.returnOrientation(self.yaw)
                # Turn 90
                giro = self.turn90(pi/2, pi/2, yawNow)

                if giro == False:
                    print ("Turn done")
                    self.firstTurn = True
                    # Go forwards
                    self.goForward(0.2)
                    time.sleep(1)
                    self.secondTurn = False

            elif self.secondTurn == False and self.crash == True:
                print ("Second turn")
                # Yaw
                yawNow = self.pose3d.getPose3d().yaw
                giro = self.turn90(pi, 0, yawNow)

                if giro == False:
                    self.secondTurn = True

            else:
                print ("Go forward...")
                # Go forward
                self.goForward(0.5)
                self.crash = False
                self.firstTurn = True

        else:
            # There is saturation
            print ("PERIMETER")

            # Get the data of the laser sensor, which consists of 180 pairs of values
            laser_data = self.laser.getLaserData()

            # Distance in millimeters, we change to cm
            laserRight = laser_data.distanceData[0]/10
            laserCenter = laser_data.distanceData[90]/10
            laser45 = laser_data.distanceData[45]/10

            # Calculate the angle of triangle
            a = self.calculateSideTriangle(laserRight, laser45, 45)
            angleC = self.calculateAngleTriangle(a, laserRight, laser45)

            # Initialize start time
            self.initPerimTime()
            timeNow = time.time()

            # Only walks the wall for a while
            if self.startTime - timeNow < self.TIME_PERIM:
                if crash == 0 and self.crashObstacle == False:
                    # I go forward until I find an obstacle
                    self.goForward(0.5)
                    print("GO FORWARD")
                elif crash == 1 and self.crashObstacle == False:
                    self.crashObstacle = True
                    print("NEW CRASH")
                    # Stop and go backwards
                    self.stopAndBackwards()

                if self.crashObstacle == True:
                    # Turn until the obstacle is to the right
                    self.turnUntilObstacleToRight(angleC)

                    if self.obstacleRight == True:
                        # The obstacle is on the right
                        if laserCenter < self.DIST_TO_OBST_FRONT or self.corner == True:
                            # Vacuum is in the corner
                            print ('Vacuum is in the corner ')
                            self.turnCorner(yaw)

                        else:
                            # Go near to the wall
                            print("Go near to the wall")
                            self.goNearToWall(laserRight)
                            self.yaw = yaw

            else:
                # Restart all global variables
                self.restartVariables()
