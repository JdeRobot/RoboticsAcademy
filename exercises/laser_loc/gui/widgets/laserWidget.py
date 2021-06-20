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
#       Carlos Awadallah Estévez<carlosawadallah@gmail.com>

from PyQt5.QtWidgets import QWidget, QGridLayout, QLabel
from PyQt5.QtGui import QPen, QPainter
from PyQt5.QtCore import QPoint, QPointF, pyqtSignal, Qt
from PyQt5 import QtGui, QtCore
import numpy as np
import math
from math import pi as pi
from math import cos, sin
import cv2

class LaserWidget(QWidget):

    stopSIG=pyqtSignal()
    
    def __init__(self,winParent):    
        super(LaserWidget, self).__init__()
        self.winParent=winParent
        self.initUI()
        self.painterScale = 15.0
        self.laser = []
        self.tLaser = []
        self.robotAngle = self.winParent.map.robotAngle
        self.scale = self.winParent.map.scale
        
    
    def initUI(self):
        layout=QGridLayout() 
        self.setLayout(layout)
        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), Qt.white)
        self.setPalette(p)
        self.resize(300,250)
        self.setMinimumSize(200,300)
        
    
    def paintEvent(self, e):
        _width = self.width()
        _height = self.height()
    
        painter=QPainter(self)
        pen = QPen(Qt.black, 2)
        painter.setPen(pen)
    
        #Widget center
        painter.translate(QPoint(_width/2, _height/2))

        # Draw laser
        colorLaser = Qt.blue
        colorTLaser = Qt.red
        yaw = self.winParent.getPose3D().yaw
        #painter.rotate(-180*yaw/pi)
        self.drawLaser(painter, colorLaser, self.laser)
        if self.tLaser != []:
            self.drawLaser(painter, colorTLaser, self.tLaser)
        # Draw axis
        #painter.rotate(180*yaw/pi)
        self.drawAxis(painter)
        # Draw Robot (triangle)
        #painter.rotate(-180*yaw/pi)
        self.drawRobot(painter)


    def drawAxis(self, painter):
        pi = math.pi
        RTx = self.RTx(pi, 0, 0, 0)
        RTz = self.RTz(pi/2, 0, 0, 0)
        RT1 = np.matrix([[0],[0],[0],[1]])
        RT2 = np.matrix([[200],[0],[0],[1]])
        RT3 = np.matrix([[0],[200],[0],[1]])
        
        RT4 = RTx  * RTz * RT1
        RT5 = RTx  * RTz * RT2
        RT6 = RTx  * RTz * RT3
       
        pen = QPen(Qt.darkGreen, 2)
        painter.setPen(pen)
        painter.drawLine(QPointF(RT4.flat[0],RT4.flat[1]),QPointF(RT5.flat[0],RT5.flat[1]))
        pen = QPen(Qt.green, 2)
        painter.setPen(pen)
        painter.drawLine(QPointF(RT4.flat[0],RT4.flat[1]),QPointF(RT6.flat[0],RT6.flat[1]))
              
    
    def RTx(self, angle, tx, ty, tz):
        RT = np.matrix([[1, 0, 0, tx], [0, math.cos(angle), -math.sin(angle), ty], [0, math.sin(angle), math.cos(angle), tz], [0,0,0,1]])
        return RT
        
    
    def RTy(self, angle, tx, ty, tz):
        RT = np.matrix([[math.cos(angle), 0, math.sin(angle), tx], [0, 1, 0, ty], [-math.sin(angle), 0, math.cos(angle), tz], [0,0,0,1]])
        return RT
    
    
    def RTz(self, angle, tx, ty, tz):
        RT = np.matrix([[math.cos(angle), -math.sin(angle), 0, tx], [math.sin(angle), math.cos(angle),0, ty], [0, 0, 1, tz], [0,0,0,1]])
        return RT 

    
    def RTLaser(self):
        # Rotation Z / Traslation X
        RT = self.RTz(-pi/2, 0, 0, 0)
        return RT
    
    
    def coordLaser(self, dist, angle):
        coord = [0,0] 
        coord[0] = dist * math.cos(angle)
        coord[1] = dist * math.sin(angle)
        return coord


    def drawRobot(self, painter):
        x = self.winParent.getPose3D().x  
        y = self.winParent.getPose3D().y    

        final_poses = self.RTRobot() * self.RTz(pi/2, 1, 0, 0)* np.matrix([[0], [0], [1], [1]]) * self.scale
        painter.translate(QPoint(final_poses[0], final_poses[1]))

        triangle = QtGui.QPolygon()

        triangle.append(QtCore.QPoint(x+51, y))
        triangle.append(QtCore.QPoint(x+71, y+35))
        triangle.append(QtCore.QPoint(x+31, y+35))

        pen = QPen(Qt.green, 2)
        painter.setPen(pen)
        painter.drawPolygon(triangle)
    
    def RTRobot(self):
        RTx = self.RTx(pi, 0, 0, 0)
        RTy = self.RTy(0, 0, 0, 0)
        RTz = self.RTz(self.robotAngle, 0, 0, 0)
        return RTz*RTy*RTx

    
    def drawLaser(self, painter, color, laser):
        pen = QPen(color, 2)
        painter.setPen(pen)
        RT = self.RTLaser()
        RTOrigLaser = np.matrix([[0],[0],[0],[1]]) * self.painterScale
        RTFinalLaser1 = RT*RTOrigLaser
        RTFinalLaser =  self.RTRobot()*RTFinalLaser1
        if (laser == self.laser):
            for d in range(0,len(laser),22):
                dist = laser[d][0]
                angle = laser[d][1]
                coord = self.coordLaser(dist,angle)
                orig_poses = np.matrix([[coord[0]], [coord[1]], [1], [1]]) * self.painterScale
                final_poses1 = RT*orig_poses
                final_poses = self.RTRobot()*final_poses1
                painter.drawLine(QPointF(RTFinalLaser.flat[0],RTFinalLaser.flat[1]),QPointF(final_poses.flat[0], final_poses.flat[1]))
        else:
            for d in range(0,len(laser)):
                dist = laser[d][0]
                angle = laser[d][1]
                coord = self.coordLaser(dist,angle)
                orig_poses = np.matrix([[coord[0]], [coord[1]], [1], [1]]) * self.painterScale
                final_poses1 = RT*orig_poses
                final_poses = self.RTRobot()*final_poses1
                painter.drawLine(QPointF(RTFinalLaser.flat[0],RTFinalLaser.flat[1]),QPointF(final_poses.flat[0], final_poses.flat[1]))
   
    def setLaserValues(self, laser):
        # Init laser array
        if len(self.laser) == 0:
            for i in range(180):
                self.laser.append((0,0))

        for i in range(180):
            dist = laser.values[i]#distanceData[i]/1000.0
            angle = -math.pi/2 + math.radians(i)
            self.laser[i] = (dist, angle)
