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
from gui.GUI import Particle
from PyQt5 import QtGui
import copy
from jderobotTypes import CMDVel

time_cycle = 10
        

class MyAlgorithm(threading.Thread):

    def __init__(self, pose3d, motors, laser, map_img):
        # Robot's sensors and actuators
        # ------------------
        self.pose3d = pose3d
        self.motors = motors
        self.laser = laser
        # ------------------
        self.map_img = map_img

        self.particles = []
        self.particleClicked = None
        self.iteration = 0
        self.numParticles = 650
        self.calculated = False;
        self.posePrev = []
        self.sent = False
        self.convergence = False;
        self.located = False;
        self.i = 0
        
        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)

    
    def parse_laser_data(self,laser_data):
        laser = []
        for i in range(180):
            dist = laser_data.values[i]
            angle = math.radians(i)
            laser += [(dist, angle)]
        return laser

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

    def setParticles (self, particles):
        gui = self.gui
        gui.setParticles(particles)

    def setEstimation (self, est):
        gui = self.gui
        if self.isAvailable(est[0],est[1]):
            gui.setEstimation(est)

    def isWhitePixel (self, x, y):
        img = self.map.pixmap.toImage()
        white = (1.0, 1.0, 1.0, 1.0)
        px,py = self.map.map2pixel((x,y))

        if (px < 0) or (py < 0) or (px >= self.map.width) or (py >= self.map.height):
            return False
        else:
            c = img.pixel(px,py)
            color = QtGui.QColor(c).getRgbF()
            if color == white:
                return True
            else:
                return False

    def doRayTracing(self, particle):
        teoricalLaser = []
        for i in range (-90,90,22): # 8 laser beams
            angle = particle.yaw+math.radians(i)
            stepx = 0.1*math.cos(angle)
            stepy = 0.1*math.sin(angle)
            x = particle.x-0.15*math.cos(particle.yaw)
            y = particle.y-0.15*math.sin(particle.yaw)

            while self.isWhitePixel(x,y):
                x = x-stepx
                y = y-stepy

            dist = math.sqrt(math.pow((x-particle.x),2)+math.pow(y-particle.y,2))
            teoricalLaser.append((dist, math.radians(i)))

        return teoricalLaser 

    def paintTheoricalLaser(self, laser):
        self.gui.map1.tLaser = laser

    def sendParticles(self):
        # Randomly sent
        particles = []
        for i in range(self.numParticles):
            randomX, randomY =  self.getRandomAvailablePosition()
            randomYaw =  random.uniform(-math.pi, math.pi)
            prob = self.calculateProb(randomX,randomY,randomYaw)
            particles.append(Particle(randomX,randomY,randomYaw,prob,self.map.robotAngle))

        return particles

    def calculateProb(self,x,y,yaw):
        # Observation Model || Health Function
        p = Particle(x,y,yaw,0.0,self.map.robotAngle)
        laserP = self.doRayTracing(p)
        error = 0.0
        realLaser = self.parse_laser_data(self.gui.getLaserData())

        for i in range(0,len(realLaser),22):
            realDist = realLaser[i][0]
            teoricalDist = laserP[i/22][0]
            error += math.sqrt(math.pow(realDist-teoricalDist,2))

        if error <= 4.5:
            prob = 1.0-(error-1.0)*0.028
        elif error > 4.5 and error <= 10.0:
            prob = 0.9-(error-4.5)*0.146
        else:
            prob = math.exp(-0.23*error)

        return min(prob, 1.0)

    def isAvailable(self, x, y):
        return self.isWhitePixel(x,y)
        
    def thermalNoise(self,prog):
        while True:
            # Thermal noise, centered in progenitors coordinates
            newX = np.random.normal(prog.x,0.1)
            newY = np.random.normal(prog.y,0.1)
            if self.isWhitePixel(newX,newY):
                break;
        newYaw = np.random.normal(prog.yaw,0.1)
        prob = self.calculateProb(newX,newY,newYaw)
        p = Particle(newX,newY,newYaw,prob,self.map.robotAngle)
        return p

    def particlesFilter(self,progenitors,flag):
        newParticles = []
        if (flag == 'copy'):  # this flag indicates that the generation provides information
            img = self.map.pixmap.toImage()
            for i in range(0,len(progenitors)):
                if progenitors[i] and self.isAvailable(progenitors[i].x, progenitors[i].y):
                    if (progenitors[i].prob >= 0.82): ## ELITISM
                        if (random.randint(0,2)>1):   # Sometimes the Progenitor Particle is Copied
                            newParticle = copy.deepcopy(progenitors[i])
                        else:                         # Other times, thermal noise is applied
                            newParticle = self.thermalNoise(progenitors[i])
                    else:           
                        # Thermal noise
                        newParticle = self.thermalNoise(progenitors[i])
                    newParticles.append(newParticle)
        elif (flag == 'resample'): # this flag indicates that the generation needs to be resampled
            elitism = []
            for i in self.particles:
                if i.prob >= 0.7:
                    elitism.append(i) ## ELITISM
            newParticles = self.sendParticles() # Resampling
            d = 0
            for el in elitism:
                d += 1
                newParticles.pop(d)
                newParticles.append(el)
            
        return newParticles

    def calculateRouletteSector(self,roulette):
        beginning = 0
        for i in range(0,len(self.particles)):
            end = beginning+self.particles[i].prob
            if roulette >= beginning and roulette < end:
                return self.particles[i]
            else:
                beginning = end
                
    def getRandomAvailablePosition(self):  
        while True:
            randomX = random.uniform(-5, -5+self.map.worldWidth)
            randomY = random.uniform(-5, -5+self.map.worldHeight)
            if self.isWhitePixel(randomX,randomY):
                break
        return randomX, randomY

    def sendGrid(self):
        # Send ParticlesIn Grid Layout
        particles = []
        # Only Near The Robot
        myX = self.pose3d.getPose3d().x+0.6
        myY = self.pose3d.getPose3d().y-0.5
        self.yawRobot =  self.pose3d.getPose3d().yaw
        for i in range(50): 
            while True:
                if self.isWhitePixel(myX,myY):
                    break
                else:
                    myY += 0.105
            prob = self.calculateProb(myX,myY,self.yawRobot)
            particles.append(Particle(myX,myY,self.yawRobot,prob,self.map.robotAngle))
            myY += 0.105    
            if myY > self.pose3d.getPose3d().y+0.5:
                myY = self.pose3d.getPose3d().y-0.5
                myX -= 0.19
        return particles    

    def changeYaw(self, yaw):
        # For debugging purposes. No more needed.
        for i in range(len(self.particles)):
            self.particles[i].yaw = yaw

    def recalculateProb(self):
        # For debugging purposes. No more needed.
        for i in range(len(self.particles)):
            prob = self.calculateProb(self.particles[i].x,self.particles[i].y,self.particles[i].yaw-math.pi)
            self.particles[i].prob = prob

    def calculateNewGeneration(self):
        print("new generation")
        if (not self.located):
            self.iteration += 1
            maxProb = 0.0;
            self.convergence = True;
            # Looking for the most likely particle
            for i in range(0,len(self.particles)):
                if (maxProb < self.particles[i].prob):
                    maxProb = self.particles[i].prob
                    center = self.particles[i] # greater prob
            # Checking if the generation converges
            for i in range(0,len(self.particles)):
                d = math.sqrt(math.pow(center.x-self.particles[i].x,2)+math.pow(center.y-self.particles[i].y,2))
                if (d > 2.0): # convergence translates into having all the particles inside a circle of radius 2
                    self.convergence = False;

            if self.convergence or self.iteration >= 30:
                # Prediction in case of convergence or confused generation
                if center.prob >= 0.9:
                    # Great Probability -> Prediction
                    self.setEstimation((center.x,center.y))
                    self.located = True;
                else:
                    # Low Probbility -> Resample
                    self.particles = self.sendParticles()
            elif center.prob > 0.99:
                # In case of total coincidence with real data -> Prediction
                self.setEstimation((center.x,center.y))
                self.particles = [center]
                print("//////////////////////////////////////////////////////") 
                print("L O C A T E D   AT   ({},{})".format(center.x,center.y))
                print("Error: {} cm".format(math.sqrt(math.pow(center.x-self.pose3d.getPose3d().x,2)+math.pow(center.y-self.pose3d.getPose3d().y,2)*100)))
                print("//////////////////////////////////////////////////////") 
                self.located = True;
            else:
                # Rest of cases: Calculate New Generation Based on the Previous
                # Computation of cumulative probability (PAC)
                pac = 0.0
                for i in range(0,len(self.particles)):
                    pac += self.particles[i].prob
                print ("pac: {}".format(pac))
                ## ROULETTE ALGORITHM
                if (pac <= 0.012*len(self.particles)):
                    # Low PAC -> Resample
                    self.particles = self.particlesFilter(None, 'resample')
                else:
                    # Acceptable PAC -> Thermal Noise
                    progenitors = []
                    for i in range(0,len(self.particles)):
                        roulette = random.uniform(0.0, pac)
                        selectedParticle = self.calculateRouletteSector(roulette)
                        progenitors.append(selectedParticle)
                    self.particles = self.particlesFilter(progenitors, 'copy')       
        else:
            # when located, restart the algorithm
            print("RESTARTING...")
            self.particles = self.sendParticles()
            self.located = False;
            self.iteration = 0
            self.convergence = False;

        self.setParticles(self.particles)
            
    def upgradePosition(self, despx, despy, desptheta):
        # Incorporating Odometry Data
        # Radius Increase
        dist = math.sqrt(math.pow(despx,2)+math.pow(despy,2))
        #  Radius Increase Sign (robot forward or backward)
        c = self.quadrant(self.pose3d.getPose3d().yaw)
        if c == 1:
            if despx > 0 and despy > 0:
                dist = -1*dist
        elif c == 2:
            if despx < 0 and despy > 0:
                dist = -1*dist
        elif c == 3:
            if despx < 0 and despy < 0: 
                dist = -1*dist
        elif c == 4:
            if despx > 0 and despy < 0:
                dist = -1*dist

        for i in self.particles[:]:

            #Straight Line:                (x0,y0) -> Point belonging the straight line (coord. particle)
            # y = y0 + (b/a)(x-x0) where <
            #                               b,a    -> Direction Vector of the straight Line (particle orientation)
            a = math.cos(i.yaw)
            b = math.sin(i.yaw)

            x0 = i.x
            y0 = i.y

            # Known the traveled distance, the coords of the particle and the eq. of the straight Line:
            # d = math.sqrt(math.pow((x0-x),2)+math.pow((y0-y0+(b/a)*(x-x0)),2)) ->
            # (2+b²/a²)x²-(4x0b²/a²)x-(d²-x0²b²/a²) = 0
            # Sol. for the Equation:
            x = ((-(math.sqrt((math.pow(a,2)*math.pow(dist,2))+(math.pow(b,2)*math.pow(dist,2))))/a)+((math.pow(b,2)*x0)/math.pow(a,2))+x0)/((math.pow(b,2)/math.pow(a,2))+1)
            if x:
                y = y0+((b/a)*(x-x0))
            else:
                print("WTF")
            
            # choosing the correct side to add the displacement to each particle
            if dist >= 0:
                i.x = x
                i.y = y
            else:
                myc = self.quadrant(i.yaw-math.pi)
                if myc == 1:
                    i.x += abs(i.x-x)
                    i.y += abs(i.y-y)
                elif myc == 2:
                    i.x -= abs(i.x-x)
                    i.y += abs(i.y-y)
                elif myc == 3:
                    i.x -= abs(i.x-x)
                    i.y -= abs(i.y-y)
                elif myc == 4:
                    i.x += abs(i.x-x)
                    i.y -= abs(i.y-y)
            # Angle Increase
            i.yaw += desptheta
            
            if not self.isWhitePixel(i.x,i.y):
                self.particles.remove(i)

    def quadrant(self, angle):
        if angle >= -math.pi and angle < -math.pi/2:
            return 1
        elif angle >= -math.pi/2 and angle < 0:
            return 2
        elif angle >= 0 and angle < math.pi/2: 
            return 3
        else:
            return 4

    def execute(self):
        
        # Send particles
        if not self.particles: 
            self.posePrev.append(self.pose3d.getPose3d().x) #Robot's movement
            self.posePrev.append(self.pose3d.getPose3d().y)
            self.posePrev.append(self.pose3d.getPose3d().yaw)
            # Random
            self.particles = self.sendParticles()
            #randomX = self.pose3d.getPose3d().x
            #randomY = self.pose3d.getPose3d().y
            #randomYaw =  self.pose3d.getPose3d().yaw
            #prob = self.calculateProb(randomX,randomY,randomYaw)
            #self.particles = [Particle(randomX,randomY,randomYaw,prob,self.map.robotAngle)]
            # Grid
            # self.particles = self.sendGrid()
        self.setParticles(self.particles)

        # Actual Pose
        x = self.pose3d.getPose3d().x
        y = self.pose3d.getPose3d().y
        yaw = self.pose3d.getPose3d().yaw

        despx = x-self.posePrev[0]
        despy = y-self.posePrev[1]
        desptheta = yaw-self.posePrev[2]
        
        if (abs(despx)>0.1 or abs(despy)>0.1 or abs(desptheta)>0.15):
            self.posePrev[0] = x
            self.posePrev[1] = y
            self.posePrev[2] = yaw
            # Incorporation of odometry data
            self.upgradePosition(despx,despy,desptheta)
            self.setParticles(self.particles)

        # New generations each 300 ms
        self.i += 1
        if self.i > 30:
            self.calculateNewGeneration()
            self.i = 0

        # Visualization of theorical laser
        if self.particleClicked is not None and not self.calculated:
            tLaser = self.doRayTracing(self.particleClicked)
            print(self.particleClicked.prob)
            self.paintTheoricalLaser(tLaser)
            self.calculated = True

