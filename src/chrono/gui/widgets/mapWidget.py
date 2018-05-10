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
#       Eduardo Perdices <eperdices@gsyc.es>
#

#import resources_rc
from PyQt5.QtWidgets import QWidget, QGridLayout, QLabel
from PyQt5.QtGui import QPen, QPainter, QColor, QPixmap, QImage, QBrush, QFont
from PyQt5.QtCore import QPoint, QPointF, pyqtSignal, Qt
import cv2
import numpy as np
import math
from math import pi as pi

WIDTH = 800
HEIGHT = 330

class MapWidget(QWidget):

    stopSIG=pyqtSignal()
    
    def __init__(self,winParent):    
        super(MapWidget, self).__init__()
        self.winParent=winParent
        self.initUI()

        self.carx = 0.0
        self.cary = 0.0
        self.phax = 0.0
        self.phay = 0.0
        self.scale = 4.0
        self.laser = []
        
    def initUI(self):
        layout=QGridLayout()  
        #self.setLayout(layout)
        #self.setAutoFillBackground(True)
        #p = self.palette()
        #p.setColor(self.backgroundRole(), QColor('#0D488A'))
        #self.setPalette(p)
        #self.map = cv2.imread("resources/monaco.png", cv2.IMREAD_GRAYSCALE)
        # cv2.imshow("@@@",self.map)
        # cv2.waitKey(0)
        #self.map = cv2.resize(self.map, (WIDTH,HEIGHT))
        #self.map = cv2.resize(self.map, (800, 668))
        # image = QImage(self.map.data, self.map.shape[1], self.map.shape[0], self.map.shape[1], QImage.Format_Indexed8);
        self.pixmap = QPixmap("resources/monaco800W.png")
        self.mapWidget = QLabel(self)
        self.mapWidget.setPixmap(self.pixmap)
        self.resize(WIDTH,HEIGHT)
        self.setMinimumSize(WIDTH,HEIGHT)

    def RTx(self, angle, tx, ty, tz):
        RT = np.matrix([[1, 0, 0, tx], [0, math.cos(angle), -math.sin(angle), ty], [0, math.sin(angle), math.cos(angle), tz], [0,0,0,1]])
        return RT
        
    def RTy(self, angle, tx, ty, tz):
        RT = np.matrix([[math.cos(angle), 0, math.sin(angle), tx], [0, 1, 0, ty], [-math.sin(angle), 0, math.cos(angle), tz], [0,0,0,1]])
        return RT
    
    def RTz(self, angle, tx, ty, tz):
        RT = np.matrix([[math.cos(angle), -math.sin(angle), 0, tx], [math.sin(angle), math.cos(angle),0, ty], [0, 0, 1, tz], [0,0,0,1]])
        return RT 

    def drawCircle(self, painter, centerX, centerY, color, size):
        pen = QPen(color, size)
        painter.setPen(pen)
        brush = QBrush(Qt.SolidPattern)
        brush.setColor(QColor(Qt.blue))
        painter.setBrush(brush)
        painter.drawEllipse(centerX, centerY, 5, 5)

    def drawName(self, painter, posx, posy, color, size, name):
        pen = QPen(color, size)
        painter.setPen(pen)
        px1 = posx - 10
        py1 = posy - 10
        # painter.drawLine(QPointF(posx - 5,posy + 5), QPointF(px1,py1))
        # painter.drawLine(QPointF(px1,py1), QPointF(px1+10,py1))
        painter.setFont(QFont("Ubuntu Mono",12, QFont.Bold))
        painter.drawText(QPointF(px1, py1), name)

        
    def paintEvent(self, e):

        copy = self.pixmap.copy()
        painter = QPainter(copy)
        painter.translate(QPoint(self.width()/2, self.height()/2))
        RTx = self.RTx(-pi, 0, 0, 0)
        p = RTx*np.matrix([[self.carx], [self.cary], [1], [1]])
        px = p.flat[0]*self.scale
        py = p.flat[1]*self.scale        
        self.drawCircle(painter,px,py,Qt.blue,2)
        self.drawName(painter,px,py,Qt.blue,1, "F1")

        #Draw phantom
        p = RTx*np.matrix([[self.phax], [self.phay], [1], [1]])
        px = p.flat[0]*self.scale
        py = p.flat[1]*self.scale        
        self.drawCircle(painter,px,py,Qt.black,2)
        self.drawName(painter,px,py,Qt.black,1, "Pha")

        self.mapWidget.setPixmap(copy)
        painter.end()


    def drawCar(self, painter):
        carsize = 60
        tiresize = carsize/5

        pen = QPen(Qt.black, 1)
        painter.setPen(pen)

        # Connectors
        painter.drawLine(QPointF(-carsize/5,carsize/5),QPointF(carsize/5, carsize/5))

        # Chassis
        painter.fillRect(-carsize/6,carsize/2,carsize/3,carsize/2,Qt.red)
        painter.fillRect(-carsize/16,0,carsize/8,carsize,Qt.red)
        painter.fillRect(-carsize/6,-carsize/24,carsize/3,carsize/12,Qt.red)
        painter.fillRect(-carsize/8,carsize-carsize/96,carsize/4,carsize/12,Qt.red)

        # Tires
        painter.fillRect(-carsize/4,carsize/8,tiresize/2,tiresize,Qt.black)
        painter.fillRect(carsize/4,carsize/8,-tiresize/2,tiresize,Qt.black)
        painter.fillRect(-carsize/4,carsize-carsize/8,tiresize/2,tiresize,Qt.black)
        painter.fillRect(carsize/4,carsize-carsize/8,-tiresize/2,tiresize,Qt.black)


    def drawLasel(self, painter):
        pen = QPen(QColor('#6897BB'), 2)
        painter.setPen(pen)
        for d in self.laser:
            px = -d[0]*math.sin(d[1])*self.scale
            py = d[0]*math.cos(d[1])*self.scale
            painter.drawLine(QPointF(0,0),QPointF(py, px))
            
    def drawArrow(self, painter, posx, posy, color, width):
        if posx == 0.0 and posy == 0.0:
            return        

        _width = self.width()
        _height = self.height()

        pen = QPen(color, width)
        painter.setPen(pen)

        # Calculate relative coordintaes of point
        #px = _width/2*posx/10.0
        #py = _height/2*posy/10.0

        RTx = self.RTx(pi, 0, 0, 0)
        RTz = self.RTz(pi/2, 0, 0, 0)
        RT = RTx*RTz
        p = RT*np.matrix([[posx], [posy], [1], [1]])
        px = p.flat[0]*self.scale
        py = p.flat[1]*self.scale

        # Draw main line
        painter.drawLine(QPointF(0,0),QPointF(px, py))
        #print("PRE: ",posx, posy)
        #print("POST: ",px,py)

        # Draw sides
        sidex = math.hypot(px, py)/5.0
        sidey = math.hypot(px, py)/5.0
        if px != 0.0:
            ang = math.atan2(py,px)
        else:
            ang = math.pi/2.0
        if posx >= 0.0:
            px1 = px + sidex * math.cos(math.pi+ang-0.5)
            py1 = py + sidey * math.sin(math.pi+ang-0.5)
            px2 = px + sidex * math.cos(math.pi+ang+0.5)
            py2 = py + sidey * math.sin(math.pi+ang+0.5)
        else:
            px1 = px - sidex * math.cos(ang-0.5)
            py1 = py - sidey * math.sin(ang-0.5)
            px2 = px - sidex * math.cos(ang+0.5)
            py2 = py - sidey * math.sin(ang+0.5)    
        painter.drawLine(QPointF(px, py),QPointF(px1, py1))
        painter.drawLine(QPointF(px, py),QPointF(px2, py2))

        #print(px,py)
        #print(px1,py1)

    def drawTarget(self, painter, posx, posy):

        if posx == 0.0 and posy == 0.0:
            return        

        pen = QPen(Qt.yellow, 4)
        painter.setPen(pen)

        sx = posx - 0.25
        sy = posy - 0.25
        ex = posx + 0.25
        ey = posy + 0.25
        painter.drawLine(QPointF(-sx*self.scale,sy*self.scale),QPointF(-ex*self.scale,ey*self.scale))
        painter.drawText( QPoint(-sx*self.scale+3,sy*self.scale), self.targetid );


        sx = posx + 0.25
        sy = posy - 0.25
        ex = posx - 0.25
        ey = posy + 0.25
        painter.drawLine(QPointF(-sx*self.scale,sy*self.scale),QPointF(-ex*self.scale,ey*self.scale))

    def setCarPos(self, x, y):
        self.carx = x
        self.cary = y

    def setPhantomPos(self, x, y):
        self.phax = x
        self.phay = y

    def setLaserValues(self, laser):
        # Init laser array
        if len(self.laser) == 0:
            for i in range(laser.numLaser):
                self.laser.append((0,0))

        for i in range(laser.numLaser):
            dist = laser.distanceData[i]/1000.0
            angle = math.radians(i)
            self.laser[i] = (dist, angle)



 
       

