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
#       Irene Lope Rodriguez<irene.lope236@gmail.com>
#       Vanessa Fernandez Martinez<vanessa_1895@msn.com>


#import resources_rc
from PyQt5.QtWidgets import QWidget, QGridLayout, QLabel
from PyQt5.QtGui import QPen, QPainter
from PyQt5.QtCore import QPoint, QPointF, pyqtSignal, Qt
from PyQt5 import QtGui, QtCore
import cv2
import numpy as np
import math
from math import pi as pi

class MapWidget(QWidget):

    stopSIG=pyqtSignal()

    def __init__(self,winParent):
        super(MapWidget, self).__init__()
        self.winParent=winParent
        self.initUI()
        self.scale = 15.0
        self.laser1 = []
        self.laser2 = []
        self.laser3 = []


    def initUI(self):
        layout=QGridLayout()
        self.setLayout(layout)
        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), Qt.white)
        self.setPalette(p)
        self.resize(300,300)
        self.setMinimumSize(500,300)


    def paintEvent(self, e):
        _width = self.width()
        _height = self.height()

        painter=QPainter(self)
        pen = QPen(Qt.black, 2)
        painter.setPen(pen)

        #Widget center
        painter.translate(QPoint(_width/2, _height/2))

        # Draw laser
        colorLaser1 = Qt.blue
        colorLaser2 = Qt.green
        colorLaser3 = Qt.red
        self.drawLaser(1, painter, colorLaser1, self.laser1)
        self.drawLaser(2, painter, colorLaser2, self.laser2)
        self.drawLaser(3, painter, colorLaser3,  self.laser3)

        # Draw car
        self.drawCar(painter)

        # Draw axis
        self.drawAxis(painter)


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

        pen = QPen(Qt.red, 2)
        painter.setPen(pen)
        painter.drawLine(QPointF(RT4.flat[0],RT4.flat[1]),QPointF(RT5.flat[0],RT5.flat[1]))
        pen = QPen(Qt.green, 2)
        painter.setPen(pen)
        painter.drawLine(QPointF(RT4.flat[0],RT4.flat[1]),QPointF(RT6.flat[0],RT6.flat[1]))


    def drawCar(self, painter):
        carsize = 40

        # Chassis
        painter.fillRect(-carsize/2, -carsize,carsize,2*carsize,Qt.yellow)

        # Tires
        painter.fillRect(-carsize/2,-carsize,carsize/5,2*carsize/5,Qt.black)
        painter.fillRect(carsize/2,-carsize,-carsize/5,2*carsize/5,Qt.black)
        painter.fillRect(-carsize/2,carsize-2*carsize/5,carsize/5,2*carsize/5,Qt.black)
        painter.fillRect(carsize/2,carsize-2*carsize/5,-carsize/5,2*carsize/5,Qt.black)


    def RTx(self, angle, tx, ty, tz):
        RT = np.matrix([[1, 0, 0, tx], [0, math.cos(angle), -math.sin(angle), ty], [0, math.sin(angle), math.cos(angle), tz], [0,0,0,1]])
        return RT


    def RTy(self, angle, tx, ty, tz):
        RT = np.matrix([[math.cos(angle), 0, math.sin(angle), tx], [0, 1, 0, ty], [-math.sin(angle), 0, math.cos(angle), tz], [0,0,0,1]])
        return RT


    def RTz(self, angle, tx, ty, tz):
        RT = np.matrix([[math.cos(angle), -math.sin(angle), 0, tx], [math.sin(angle), math.cos(angle),0, ty], [0, 0, 1, tz], [0,0,0,1]])
        return RT


    def RTLaser(self, num):
        if num == 1:
            # Rotation Z / Traslation X
            RT = self.RTz(0, 2.79, 0, 0)
        elif num == 2:
            # Rotation Z / Traslation X
            RT = self.RTz(pi, -2.79, 0, 0)
        else:
            # Rotation Z / Traslation Y
            RT = self.RTz(-pi/2, 0, -1.5, 0)
        return RT


    def coordLaser(self, dist, angle):
        coord = [0,0]
        coord[0] = dist * math.cos(angle)
        coord[1] = dist * math.sin(angle)
        return coord


    def RTCar(self):
        RTx = self.RTx(pi, 0, 0, 0)
        RTz = self.RTz(pi/2, 0, 0, 0)
        return RTx*RTz


    def drawLaser(self, num, painter, color, laser):
        pen = QPen(color, 2)
        painter.setPen(pen)
        RT = self.RTLaser(num)
        RTOrigLaser = np.matrix([[0],[0],[0],[1]]) * self.scale
        RTFinalLaser1 = RT * RTOrigLaser
        RTFinalLaser = self.RTCar() * RTFinalLaser1
        for d in laser:
            dist = d[0]
            angle = d[1]
            coord = self.coordLaser(dist,angle)
            orig_poses = np.matrix([[coord[0]], [coord[1]], [1], [1]]) * self.scale
            final_poses1 = RT * orig_poses
            final_poses = self.RTCar() * final_poses1
            painter.drawLine(QPointF(RTFinalLaser.flat[0],RTFinalLaser.flat[1]),QPointF(final_poses.flat[0], final_poses.flat[1]))


    def setLaserValues(self, num, laser):
        # Init laser array
        if num == 1:
            laserX = self.laser1
        elif num == 2:
            laserX = self.laser2
        else:
            laserX = self.laser3

        if len(laserX) == 0:
            for i in range(len(laser.values)):
                laserX.append((0,0))

        for i in range(len(laser.values)):
            dist = laser.values[i]/1000.0
            angle = -math.pi/2 + math.radians(i)
            laserX[i] = (dist, angle)


class MapWidget1(QWidget):

    stopSIG=pyqtSignal()

    def __init__(self,winParent):
        super(MapWidget1, self).__init__()
        self.winParent=winParent
        self.initUI()
        self.scale = 12.0
        self.trail = []


    def initUI(self):
        layout=QGridLayout()
        self.setLayout(layout)
        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), Qt.white)
        self.setPalette(p)
        self.resize(300,300)
        self.setMinimumSize(500,300)


    def paintEvent(self, e):
        _width = self.width()
        _height = self.height()

        painter2=QPainter(self)
        pen = QPen(Qt.green, 2)
        painter2.setPen(pen)

        # Widget center
        painter2.translate(QPoint(_width/2, _height/2))

        # Draw obstacles
        self.drawObstacles(painter2)

        # Draw ideal position
        self.drawIdeal(painter2)

        painter=QPainter(self)
        pen = QPen(Qt.black, 2)
        painter.setPen(pen)

        # Widget center
        painter.translate(QPoint(_width/2, _height/2))

        # Draw car
        self.drawCar(painter)

        painter1=QPainter(self)
        pen = QPen(Qt.red, 2)
        painter1.setPen(pen)

        # Widget center
        painter1.translate(QPoint(_width/2, _height/2))

        # Draw the car's way
        self.drawTrail(painter1)


    def RTCar(self):
        RTx = self.RTx(pi, 0, 0, 0)
        RTz = self.RTz(pi/2, 0, 0, 0)
        return RTx*RTz


    def drawCar(self, painter):
        pose = self.winParent.getPose3D().getPose3d()
        x = pose.x
        y = pose.y
        yaw = pose.yaw

        orig_poses = np.matrix([[x], [y], [1], [1]]) * self.scale
        final_poses = self.RTCar() * orig_poses

        carsize = 25
        painter.translate(QPoint(final_poses[0],final_poses[1]))
        painter.rotate(-180*yaw/pi)

        # Chassis
        painter.fillRect(-carsize/2, -carsize,carsize,2*carsize,Qt.yellow)

        # Tires
        painter.fillRect(-carsize/2,-carsize,carsize/5,2*carsize/5,Qt.black)
        painter.fillRect(carsize/2,-carsize,-carsize/5,2*carsize/5,Qt.black)
        painter.fillRect(-carsize/2,carsize-2*carsize/5,carsize/5,2*carsize/5,Qt.black)
        painter.fillRect(carsize/2,carsize-2*carsize/5,-carsize/5,2*carsize/5,Qt.black)


    def drawTrail(self, painter):
        pose = self.winParent.getPose3D().getPose3d()
        x = pose.x
        y = pose.y
        yaw = pose.yaw

        orig_poses = np.matrix([[x], [y], [1], [1]]) * self.scale
        final_poses = self.RTCar() * orig_poses

        if len(self.trail) < 300:
            self.trail.append([final_poses.flat[0], final_poses.flat[1]])
        else:
            for i in range(1, len(self.trail)):
                self.trail[i-1] = self.trail[i]
            self.trail[len(self.trail)-1] = [final_poses.flat[0], final_poses.flat[1]]

        for i in range(0, len(self.trail)):
            painter.drawPoint(self.trail[i][0], self.trail[i][1])


    def drawObstacles(self, painter):
        carsize = 30

        # Obstacle 1
        orig_poses1 = np.matrix([[-13.5], [-3], [1], [1]]) * self.scale
        final_poses1 = self.RTCar() * orig_poses1
        painter.fillRect(-carsize/2+final_poses1.flat[0], -carsize+final_poses1.flat[1], carsize, 2*carsize, Qt.black)
        # Obstacle 2
        orig_poses2 = np.matrix([[-7], [-3], [1], [1]]) * self.scale
        final_poses2 = self.RTCar() * orig_poses2
        painter.fillRect(-carsize/2+final_poses2.flat[0], -carsize+final_poses2.flat[1], carsize, 2*carsize, Qt.black)
        # Obstacle 3
        orig_poses3 = np.matrix([[0.5], [-3], [1], [1]]) * self.scale
        final_poses3 = self.RTCar() * orig_poses3
        painter.fillRect(-carsize/2+final_poses3.flat[0], -carsize+final_poses3.flat[1], carsize, 2*carsize, Qt.black)
        # Obstacle 4
        orig_poses4 = np.matrix([[14], [-3], [1], [1]]) * self.scale
        final_poses4 = self.RTCar() * orig_poses4
        painter.fillRect(-carsize/2+final_poses4.flat[0], -carsize+final_poses4.flat[1], carsize, 2*carsize, Qt.black)

        # Sidewalk 1
        orig_poses5 = np.matrix([[5], [9], [1], [1]]) * self.scale
        final_poses5 = self.RTCar() * orig_poses5
        painter.fillRect(-5*carsize+final_poses5.flat[0], -6*carsize+final_poses5.flat[1], 6.75*carsize, 16*carsize, Qt.black)
        # Sidewalk 2
        orig_poses6 = np.matrix([[5], [-9], [1], [1]]) * self.scale
        final_poses6 = self.RTCar() * orig_poses6
        painter.fillRect(-1.8*carsize+final_poses6.flat[0], -6*carsize+final_poses6.flat[1], 6.75*carsize, 16*carsize, Qt.black)


    def drawIdeal(self, painter):
        carsize = 30

        # Ideal position
        orig_poses = np.matrix([[7.25], [-3], [1], [1]]) * self.scale
        final_poses = self.RTCar() * orig_poses
        painter.drawLine(-carsize/2+final_poses.flat[0], -carsize+final_poses.flat[1], carsize/2+final_poses.flat[0], -carsize+final_poses.flat[1])
        painter.drawLine(carsize/2+final_poses.flat[0], -carsize+final_poses.flat[1], carsize/2+final_poses.flat[0], carsize+final_poses.flat[1])
        painter.drawLine(carsize/2+final_poses.flat[0], carsize+final_poses.flat[1], -carsize/2+final_poses.flat[0], carsize+final_poses.flat[1])
        painter.drawLine(-carsize/2+final_poses.flat[0], carsize+final_poses.flat[1], -carsize/2+final_poses.flat[0], -carsize+final_poses.flat[1])


    def RTx(self, angle, tx, ty, tz):
        RT = np.matrix([[1, 0, 0, tx], [0, math.cos(angle), -math.sin(angle), ty], [0, math.sin(angle), math.cos(angle), tz], [0,0,0,1]])
        return RT

    def RTy(self, angle, tx, ty, tz):
        RT = np.matrix([[math.cos(angle), 0, math.sin(angle), tx], [0, 1, 0, ty], [-math.sin(angle), 0, math.cos(angle), tz], [0,0,0,1]])
        return RT

    def RTz(self, angle, tx, ty, tz):
        RT = np.matrix([[math.cos(angle), -math.sin(angle), 0, tx], [math.sin(angle), math.cos(angle),0, ty], [0, 0, 1, tz], [0,0,0,1]])
        return RT



class LogoWidget(QWidget):
    stopSIG=pyqtSignal()

    def __init__(self,winParent):
        super(LogoWidget, self).__init__()
        self.winParent=winParent
        self.initUI()


    def initUI(self):
        self.logo = cv2.imread("resources/logo_jderobot1.png",cv2.IMREAD_UNCHANGED)
        self.logo = cv2.resize(self.logo, (100, 100))
        image = QtGui.QImage(self.logo.data, self.logo.shape[1], self.logo.shape[0], QtGui.QImage.Format_ARGB32);
        self.pixmap = QtGui.QPixmap.fromImage(image)
        self.height = self.pixmap.height()
        self.width = self.pixmap.width()
        self.logoWidget = QLabel(self)
        self.logoWidget.setPixmap(self.pixmap)
        self.logoWidget.resize(self.width, self.height)

        self.resize(300,300)
        self.setMinimumSize(100,100)
