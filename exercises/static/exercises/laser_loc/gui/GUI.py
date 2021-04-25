#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Copyright (C) 1997-2016 JDE Developers Team
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
#       Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
#       Carlos Awadallah Estévez <carlosawadallah@gmail.com>
#

from gui.widgets.teleopWidget import TeleopWidget
from gui.widgets.mapWidget import MapWidget
from gui.widgets.laserWidget import LaserWidget
from gui.widgets.mapWidget import LogoWidget
from PyQt5.QtCore import pyqtSignal
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QMainWindow
from gui.form import Ui_MainWindow
import math
import time
from jderobotTypes import CMDVel

class Particle:
    def __init__(self, x, y, yaw, prob, mapAngle):
        self.x = x
        self.y=y
        self.yaw=mapAngle+yaw
        self.prob=prob

class MainWindow(QMainWindow, Ui_MainWindow):

    updGUI=pyqtSignal()
    def __init__(self, map_img, parent=None):
        super(MainWindow, self).__init__(parent)
        self.teleop=TeleopWidget(self)
        self.map_img = map_img
        self.map=MapWidget(self)
        self.setupUi(self, self.map.width, self.map.height)
        self.map1=LaserWidget(self)
        self.logo = LogoWidget(self, "resources/logo_jderobot1.png")
        self.logo2 = LogoWidget(self, "resources/logo_jderobot2.png")
        self.tlLayout.addWidget(self.teleop)
        self.teleop.setVisible(True)
        self.mapLayout.addWidget(self.map)
        self.map1Layout.addWidget(self.map1)
        self.logoLayout.addWidget(self.logo)
        self.roboticsLogoLayout.addWidget(self.logo2)
        self.map.setVisible(True)
        self.map1.setVisible(True)
        self.logo.setVisible(True)

        #self.rotationDial.valueChanged.connect(self.rotationChange)
        #self.probButton.clicked.connect(self.recalculate)
        #self.probButton.setCheckable(True)
        #self.newGeneration.clicked.connect(self.nextGeneration)
        #self.newGeneration.setCheckable(True)
        self.pushButton.clicked.connect(self.playClicked)
        self.pushButton.setCheckable(True)
        self.updGUI.connect(self.updateGUI)

        self.particles = []
        self.estimation = []

    def updateGUI(self):
        laserdata = self.sensors.laserdata
        if (laserdata):
            self.map1.setLaserValues(laserdata)
        self.map.update()
        self.map1.update()
    
    def setSensors(self, sensors):
        ''' Declares the Sensors object and its corresponding control thread. '''
        self.sensors = sensors

    def setAlgorithm(self, algorithm ):
        self.algorithm=algorithm
        algorithm.gui = self
        algorithm.map = self.map

    def getAlgorithm(self):
        return self.algorithm

    def getPose3D(self):
        return self.sensors.actualPose

    def getLaserData(self):
        return self.sensors.laserdata

    def getMapImg(self):
        return self.map_img

    def setMapImg(self,map_img):
        self.map_img=map_img

    def getParticles(self):
        return self.particles

    def setParticles(self,particles):
        self.particles=particles

    def getEstimation(self):
        return self.estimation

    def setEstimation(self,est):
        if len(self.estimation) >= 15:
            self.estimation.pop(0)

        self.estimation.append(est)

    def playClicked(self):
        if self.pushButton.isChecked():
            self.pushButton.setStyleSheet("background-color: #ec7063")
            icon = QtGui.QIcon()
            icon.addPixmap(QtGui.QPixmap(":/images/stop.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.pushButton.setIcon(icon)
            self.algorithm.play()
        else:
            self.sensors.motors.sendV(0)
            self.sensors.motors.sendW(0)
            icon = QtGui.QIcon()
            self.pushButton.setStyleSheet("background-color: #7dcea0")
            icon.addPixmap(QtGui.QPixmap(":/images/play.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.pushButton.setIcon(icon)
            self.algorithm.stop()
            self.teleop.returnToOrigin()

    def setAlgorithm(self, algorithm ):
        self.algorithm=algorithm
        algorithm.gui = self
        algorithm.map = self.map

    def getAlgorithm(self):
        return self.algorithm

    def setXYValues(self,newX,newY):
        vel = CMDVel()
        myW=-newX*(self.sensors.motors.getMaxW())*2
        myV=-newY*(self.sensors.motors.getMaxV())
        vel.vx = myV
        vel.az = myW
        self.sensors.motors.sendVelocities(vel)
        #self.sensors.motors.sendV(myV)
        #self.sensors.motors.sendW(myW)

    def rotationChange(self,value):
        val = ((value*2*math.pi)/360)# - (self.rotationDial.maximum()/2))
        #self.rotValue.setText('%.2f' % value)
        if (self.algorithm):
            self.algorithm.changeYaw(val)

    def recalculate(self):
        if (self.algorithm):
            self.algorithm.recalculateProb()

    def nextGeneration(self):
        if (self.algorithm):
            self.algorithm.calculateNewGeneration()

