from sensors import sensor
import numpy as np
import cv2
import math

import threading
import time
from datetime import datetime
#from ompl_solution.OptimalPlanning import optimalPlanning
from ompl_solution.Point2DPlanning import Plane2DEnvironment
import sys

time_cycle = 80

class MyAlgorithm(threading.Thread):

    def __init__(self, grid, sensor, vel):
        self.sensor = sensor
        self.grid = grid
        self.vel = vel
        sensor.getPathSig.connect(self.generatePath)

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)


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

    """ Write in this method the code necessary for looking for the shorter
        path to the desired destiny.
        The destiny is chosen in the GUI, making double click in the map.
        This method will be call when you press the Generate Path button. 
        Call to grid.setPath(path) mathod for setting the path. """
    def generatePath(self):
        print("LOOKING FOR SHORTER PATH")
        mapIm = self.grid.getMap()      
        dest = self.grid.getDestiny()   
        gridPos = self.grid.getPose()

        size = mapIm.shape
        h = size[0]
        w = size[1]
        print (h)
        print (w)
        mapppm = np.zeros((h,w,3))
        kernel = np.ones((5,5),np.uint8)    
        erosion = cv2.erode(mapIm,kernel,iterations = 1) 
        #cv2.imshow('image',erosion)
        for i in range(h):
            for j in range(w):
                mapppm[i,j,0] = erosion[i,j]
                mapppm[i,j,1] = erosion[i,j]
                mapppm[i,j,2] = erosion[i,j]

        
        cv2.imwrite('map_image.ppm',mapppm)

        planner = str("rrtstar")
        #objective = str(list[1])
        runtime = str(10)

        print planner
        #print objective
        print runtime

        fname = sys.path[0]+'/map_image.ppm'
        print fname
        env = Plane2DEnvironment(fname)

        if env.plan(gridPos[0],gridPos[1], dest[0],dest[1],planner,float(runtime)):
            env.recordSolution()
            env.save("result_demo.ppm")
            pathlist = [[] for i in range(2)]
            pathlist = env.getPath()

            patharray = np.array(pathlist)
            patharray = np.rint(patharray)
            #print patharray
            size = patharray.shape
            #print size
            num = 0
            pathX_old = -1
            pathY_old = -1
            for i in range(size[1]):
                pathX = patharray[0][i]
                pathY = patharray[1][i]
                if pathX != pathX_old or pathY != pathY_old:
                    self.grid.setPathVal (int(pathX), int(pathY), num)
                    num += 1
                    pathX_old = pathX
                    pathY_old = pathY
                    # print pathX
                    # print pathY
                    # print num

            self.grid.setPathFinded()
        #Represent the Gradient Field in a window using cv2.imshow

    """ Write in this mehtod the code necessary for going to the desired place,
        once you have generated the shorter path.
        This method will be periodically called after you press the GO! button. """

    def findTargetPose(self,x,y):
        pathMap = self.grid.getPath()
        gridPose = self.grid.worldToGrid(y,x)
        size = pathMap.shape
        maxNum = np.max(pathMap)
        print maxNum
        h = size[0]
        w = size[1]
        r = 5
        findPose = False
        TargetPose = [0,0,0]
        MaxVal = 0
        print gridPose

        while not findPose:
            for i in range(-r,r):
                for j in range(-r+abs(i),r-abs(i)):
                    poseX = gridPose[0] + i
                    poseY = gridPose[1] + j
                    if poseX<h and poseY<w and poseX>0 and poseY>0:
                        tmpVal = pathMap[poseX][poseY]
                        if tmpVal > MaxVal:
                            TargetPose[0] = poseX
                            TargetPose[1] = poseY
                            MaxVal = tmpVal
                            # pathMap[poseX][poseY] = 0
            if MaxVal>0:
                print MaxVal
                print TargetPose
                print ("find target pose")
                if MaxVal == maxNum:
                    TargetPose[2] = 1
                findPose = True
            else:
                r += 2
            if r > h:
                findPose = True
                print ("didn't find target pose")
        worldTargetPose = self.grid.gridToWorld(TargetPose[1],TargetPose[0])
        return [worldTargetPose[0],worldTargetPose[1],TargetPose[2]]

    def execute(self):
        # Add your code here
        print("GOING TO DESTINATION")

        x = self.sensor.getRobotX()
        y = self.sensor.getRobotY()
        theta = self.sensor.getRobotTheta()

        TargetPose = self.findTargetPose(x,y)
        yaw = math.atan2(TargetPose[1]-y,TargetPose[0]-x)
        yaw_dis = yaw - theta
        dis = math.sqrt(pow(x-TargetPose[0],2)+pow(y-TargetPose[1],2))

        if yaw_dis > 3.14:
            yaw_dis -= 6.28
        if yaw_dis < -3.14:
            yaw_dis += 6.28 

        if yaw_dis > 1 or yaw_dis < -1:
            v = 0
            w = yaw_dis
        else:
            v = dis*1
            if v<0.1:
                v = 0.1
            w = yaw_dis
        
        print TargetPose
        # self.vel.setV(v)
        print x
        print y
        print theta
        print yaw_dis

        if (TargetPose[2] == 1):
            if dis<1:
                print "get target"
                v = 0
                w = 0
        
        self.vel.setV(v)
        self.vel.setW(w)

        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
