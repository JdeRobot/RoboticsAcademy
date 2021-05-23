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


from PyQt5.QtWidgets import QWidget, QGridLayout, QLabel
from PyQt5.QtGui import QPen, QPainter
from PyQt5.QtCore import QPoint, QPointF, pyqtSignal, Qt
from PyQt5 import QtGui, QtCore
import numpy as np
import math
from math import pi as pi
from math import cos, sin
import cv2
import config, sys


class MapWidget(QWidget):

    stopSIG=pyqtSignal()
    
    def __init__(self,winParent):    
        super(MapWidget, self).__init__()
        self.winParent=winParent
        self.initUI()
        self.laser = []
        self.trail = []
        self.trajectory = [];
        self.prevx = -10;
        self.prevy = -10;
        self.particlesPrev = []
        
    
    def initUI(self):
        self.initMap()
        image = QtGui.QImage(self.map.data, self.map.shape[1], self.map.shape[0], self.map.shape[1], QtGui.QImage.Format_Indexed8);
        self.pixmap = QtGui.QPixmap.fromImage(image)
        
        self.height = self.pixmap.height()
        self.width = self.pixmap.width()
        self.mapWidget = QLabel(self)
        self.mapWidget.setPixmap(self.pixmap)
        self.mapWidget.resize(self.width, self.height)
        self.mapWidget.mousePressEvent = self.getPixelClicked

        if self.width < 400 or self.width > 630 or self.height < 400 or self.height > 680:
            raise DimensionsError()
        else:
            #self.resize(300,300)
            self.setMinimumSize(self.width, self.height)

    def initMap(self):
        self.map = cv2.imread(self.winParent.map_img, cv2.IMREAD_GRAYSCALE)
        #self.map = cv2.resize(self.map, (500, 550))

        cfg = config.load(sys.argv[1])
        self.scale = cfg.getProperty("LaserLoc.Map.Scale")
        self.robotAngle = math.radians(cfg.getProperty("LaserLoc.Map.Angle"))
        self.worldHeight = cfg.getProperty("LaserLoc.Map.WorldHeight")
        self.worldWidth = cfg.getProperty("LaserLoc.Map.WorldWidth")
        self.originX = cfg.getProperty("LaserLoc.Map.OriginX")
        self.originY = cfg.getProperty("LaserLoc.Map.OriginY")

    def map2pixel(self, coordinates):
        px = 250-coordinates[0]*self.scale#+30  ##276
        py = coordinates[1]*self.scale+275     ##240
        return px,py
     

    def pixel2map(self, pixel):
        x = (250-pixel[0])/self.scale
        y = (pixel[1]-275)/self.scale
        return x,y

    def getPixelClicked(self, event):
        # get the pixel clicked on the map
        x = event.pos().x()
        y = event.pos().y()
        pixelClicked = x,y
        print "pixel clicked =", pixelClicked
        #determines the nearest particle
        particle = self.particleClicked(pixelClicked)
        self.winParent.algorithm.particleClicked = particle
        self.winParent.algorithm.calculated = False

    def particleClicked(self, px):
        # To obtain the nearest particle to the pixel clicked
        particles = self.winParent.getParticles()
        nearestParticle = None
        minDistance = 1000000
        x,y = self.pixel2map((px[0],px[1]))
        for particle in particles:
            dx = x - particle.x
            dy = y - particle.y
            distance = math.sqrt((dx*dx)+(dy*dy))
            if distance <= minDistance and distance <= 0.2:
                nearestParticle = particle
                minDistance = distance
            
        return nearestParticle

    def RTx(self, angle, tx, ty, tz):
        RT = np.matrix([[1, 0, 0, tx], [0, math.cos(angle), -math.sin(angle), ty], [0, math.sin(angle), math.cos(angle), tz], [0,0,0,1]])
        return RT
        
    
    def RTy(self, angle, tx, ty, tz):
        RT = np.matrix([[math.cos(angle), 0, math.sin(angle), tx], [0, 1, 0, ty], [-math.sin(angle), 0, math.cos(angle), tz], [0,0,0,1]])
        return RT
    
    
    def RTz(self, angle, tx, ty, tz):
        RT = np.matrix([[math.cos(angle), -math.sin(angle), 0, tx], [math.sin(angle), math.cos(angle),0, ty], [0, 0, 1, tz], [0,0,0,1]])
        return RT

    def RTRobot(self):
        RTy = self.RTy(self.robotAngle, 0.0, 0.0, 0)
        return RTy

    def drawRobot(self, painter):
        pose = self.winParent.getPose3D()
        x = pose.x
        y = pose.y
        yaw = pose.yaw+self.robotAngle
        pos = self.RTRobot() * np.matrix([[x], [y], [1], [1]]) * self.scale
        #painter.rotate(-180*yaw/pi)
        self.drawParticle(painter, pos[0], pos[1], yaw, -1.0)

    def drawTrajectory(self, painter):
        pose = self.winParent.getPose3D()
        actualx = pose.x
        actualy = pose.y
        actualpos = self.RTRobot() * np.matrix([[actualx], [actualy], [1], [1]]) * self.scale
        
        despx = abs(self.prevx - actualpos[0])
        despy = abs(self.prevy - actualpos[1])
        if (despx > 0.5 or despy > 0.5):
            self.trajectory.append([actualpos[0],actualpos[1]])
            self.prevx = actualpos[0]
            self.prevy = actualpos[1]

        if (len(self.trajectory)>100):
            self.trajectory.pop(0)

        color = QtGui.QColor(0,0,0)
        pen = QPen(color, 2)

        painter.setPen(pen)
        brush = QtGui.QBrush(QtCore.Qt.SolidPattern)
        brush.setColor(color)
        painter.setBrush(brush)
        d=2
        for coord in self.trajectory:
            painter.drawEllipse(QPoint(coord[0], coord[1]), d, d)

    def drawEstimatedTrajectory(self, painter):
        est = self.winParent.getEstimation()
        color = QtGui.QColor(103,226,238)
        pen = QPen(color, 2)

        painter.setPen(pen)
        brush = QtGui.QBrush(QtCore.Qt.SolidPattern)
        brush.setColor(color)
        painter.setBrush(brush)
        d=3
        for i in range(0,len(est)):
            pos = self.RTRobot() * np.matrix([[est[i][0]], [est[i][1]], [1], [1]]) * self.scale
            if i == 0:
                painter.drawEllipse(QPoint(pos[0], pos[1]), d, d)
            else:
                painter.drawEllipse(QPoint(pos[0], pos[1]), d, d) 
                painter.drawLine(pos[0],pos[1],coordPrev[0],coordPrev[1])
            coordPrev = pos
            
    def prob2Color(self, prob):
        r = 0
        g = 0
        b = 0
        if (prob >= 0.0):
            r = 255
            g = 248-round(247*prob)
        # Robot
        if (prob < 0.0):
            b = 255

        return QtGui.QColor(r,g,b)

    def drawParticle(self, painter, centerX, centerY, yaw, prob):
        color = self.prob2Color(prob)
        pen = QPen(color, 2)

        painter.setPen(pen)
        brush = QtGui.QBrush(QtCore.Qt.SolidPattern)
        brush.setColor(color)
        painter.setBrush(brush)
        d=3

        painter.drawLine(centerX,centerY,(10*cos(-yaw))+centerX,(10*sin(-yaw))+centerY)
        painter.drawEllipse(QPoint(centerX, centerY), d, d)


    def drawParticles(self, painter):
        particles = self.winParent.getParticles()
        for p in particles: 
            if (p.x > -5 and p.x < 5) and (p.y > -5 and p.y < 5):
                pos = self.RTRobot() * np.matrix([[p.x], [p.y], [1], [1]]) * self.scale
                self.drawParticle(painter, pos[0], pos[1], p.yaw, p.prob)


    def paintEvent(self, e):
        copy = self.pixmap.copy()
        painter = QtGui.QPainter(copy)

        painter.translate(QPoint(self.width/2, self.height/2))
        self.drawTrajectory(painter)
        self.drawParticles(painter)
        self.drawRobot(painter)
        self.drawEstimatedTrajectory(painter)
        self.mapWidget.setPixmap(copy)
        painter.end()
            
                   
class LogoWidget(QWidget):
    stopSIG=pyqtSignal()
    
    def __init__(self,winParent,img):    
        super(LogoWidget, self).__init__()
        self.winParent=winParent
        self.initUI(img)
        
    
    def initUI(self, img):
        self.logo = cv2.imread(img, cv2.IMREAD_UNCHANGED)
        self.logo = cv2.resize(self.logo, (80, 80))
        image = QtGui.QImage(self.logo.data, self.logo.shape[1], self.logo.shape[0], QtGui.QImage.Format_ARGB32);
        self.pixmap = QtGui.QPixmap.fromImage(image)
        self.height = self.pixmap.height()
        self.width = self.pixmap.width()
        self.logoWidget = QLabel(self)
        self.logoWidget.setPixmap(self.pixmap)
        self.logoWidget.resize(self.width, self.height)

        self.resize(300,300)
        self.setMinimumSize(100,100)

class DimensionsError(Exception):
    def __init__(self):
        self.value = "Bad dimensions for map image. Use: from 400x400 to 630x680 images"
    def __str__(self):
        return repr(self.value)


