#
#  Copyright (C) 1997-2015 JDE Developers Team
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses/.
#  Authors :
#       Alberto Martin Florido <almartinflorido@gmail.com>
#
import sys, traceback, Ice
import easyiceconfig as EasyIce
import jderobot
import numpy as np
import threading
from math import asin, atan2, pi

class Sensor:

    def __init__(self):
        self.lock = threading.Lock()
        self.playButton=False

        try:
            ic = EasyIce.initialize(sys.argv)
            properties = ic.getProperties()

            self.robot = properties.getProperty("TeleTaxi.robot")
            print "Robot:", self.robot

            motorsBase = ic.propertyToProxy("TeleTaxi.motors.Proxy")
            self.motorsProxy = jderobot.MotorsPrx.checkedCast(motorsBase)
            if self.motorsProxy:
                print 'Motors connected'
            else:
                print 'Motors not connected'

            basepose3D = ic.propertyToProxy("TeleTaxi.Pose3D.Proxy")
            self.pose3DProxy = jderobot.Pose3DPrx.checkedCast(basepose3D)
            if self.pose3DProxy:
                self.pose = jderobot.Pose3DData()
                self.angle = self.quat2Angle(self.pose.q0, self.pose.q1, self.pose.q2, self.pose.q3)
                print "Pose3D connected"
            else:
                print 'Pose3D not connected'

            self.maxSpeedV = 50
            self.maxSpeedW = 20   

        except:
            traceback.print_exc()
            exit()
            status = 1

      
    def setGrid(self, grid):
        self.grid = grid
        if self.pose3DProxy:
            self.grid.initPose(self.pose.x, self.pose.y, self.angle)
        else:
            self.grid.initPose(0, 0, 0)

    def quat2Angle(self, qw, qx, qy, qz):
        rotateZa0=2.0*(qx*qy + qw*qz)
        rotateZa1=qw*qw + qx*qx - qy*qy - qz*qz
        rotateZ=0.0
        if(rotateZa0 != 0.0 and rotateZa1 != 0.0):
            rotateZ=atan2(rotateZa0,rotateZa1)
        return rotateZ


    def update(self):
        self.lock.acquire()
        self.updatePose()
        self.lock.release()

    def updatePose(self):
        if self.pose3DProxy:
            self.pose = self.pose3DProxy.getPose3DData()
            self.angle = self.quat2Angle(self.pose.q0, self.pose.q1, self.pose.q2, self.pose.q3)
            self.grid.updatePose(self.pose.x, self.pose.y, self.angle)


    def setV(self, v, percentage=False):
        myV = v

        if (percentage):
            myV = myV * self.maxSpeedV

        if self.motorsProxy:
            self.motorsProxy.setV(myV)
        
    def setGetPathSignal(self, signal):
        self.getPathSig = signal

    def setW(self, w, percentage=False):
        myW = w

        if (percentage):
            myW = myW * self.maxSpeedW

        if self.motorsProxy:
            self.motorsProxy.setW(myW)
    
    def  getV(self):
        if self.motorsProxy:
            self.motorsProxy.getV()

    def getW(self):
        if self.motorsProxy:
            self.motorsProxy.getW()   

    def stop(self):
        if self.motorsProxy:
            self.motorsProxy.setV(0)
            self.motorsProxy.setW(0)

    def getPose3D(self):
        if self.pose3DProxy:
            self.lock.acquire()
            tmp = self.pose
            self.lock.release()
            return tmp

        return None

    def getRobotX(self):
        self.lock.acquire()
        tmp = self.pose.x
        self.lock.release()
        return tmp

    def getRobotY(self):
        self.lock.acquire()
        tmp = self.pose.y
        self.lock.release()
        return tmp

    def getRobotTheta(self):
        self.lock.acquire()
        tmp = self.angle
        self.lock.release()
        return tmp        

    def isPlayButton(self):
        return self.playButton
    
    def setPlayButton(self,value):
        self.playButton=value