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
from PyQt5.QtWidgets import QWidget, QGridLayout
from PyQt5.QtGui import QPen, QPainter, QColor
from PyQt5.QtCore import QPoint, QPointF, pyqtSignal, Qt
import numpy as np
import math
from math import pi as pi

class MapWidget(QWidget):

    stopSIG=pyqtSignal()
    
    def __init__(self,winParent):    
        super(MapWidget, self).__init__()
        self.winParent=winParent
        self.initUI()

        self.carx = 0.0
        self.cary = 0.0
        self.obsx = 0.0
        self.obsy = 0.0
        self.avgx = 0.0
        self.avgy = 0.0
        self.targetx = 0.0
        self.targety = 0.0
        self.targetid = "NaN"
        self.scale = 30.0
        self.laser = []
        self.MAX_RANGE = 0
        
    def initUI(self):
        layout=QGridLayout()  
        self.setLayout(layout)
        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), QColor('#0D488A'))
        self.setPalette(p)
        self.resize(600,600)
        self.setMinimumSize(600,600)

    def RTx(self, angle, tx, ty, tz):
        RT = np.matrix([[1, 0, 0, tx], [0, math.cos(angle), -math.sin(angle), ty], [0, math.sin(angle), math.cos(angle), tz], [0,0,0,1]])
        return RT
        
    def RTy(self, angle, tx, ty, tz):
        RT = np.matrix([[math.cos(angle), 0, math.sin(angle), tx], [0, 1, 0, ty], [-math.sin(angle), 0, math.cos(angle), tz], [0,0,0,1]])
        return RT
    
    def RTz(self, angle, tx, ty, tz):
        RT = np.matrix([[math.cos(angle), -math.sin(angle), 0, tx], [math.sin(angle), math.cos(angle),0, ty], [0, 0, 1, tz], [0,0,0,1]])
        return RT 
        
    def paintEvent(self, e):
        _width = self.width()
        _height = self.height()
    
        painter=QPainter(self)
        pen = QPen(Qt.blue, 2)
        painter.setPen(pen)
    
        #Widget center
        painter.translate(QPoint(_width/2, _height/1.5))
           
        # Draw car
        self.drawCar(painter)
        
        # Draw laser
        self.drawLaser(painter)

        # Draw target
        self.drawTarget(painter, self.targetx, self.targety)

        # Draw arrows
        self.drawArrow(painter, self.carx, self.cary, Qt.green, 2)
        self.drawArrow(painter, self.obsx, self.obsy, Qt.red, 2)
        self.drawArrow(painter, self.avgx, self.avgy, Qt.black, 2)

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


    def drawLaser(self, painter):
        pen = QPen(QColor('#6897BB'), 2)
        painter.setPen(pen)

        for d in self.laser:
            if d[0] > self.MAX_RANGE:
                px = -self.MAX_RANGE*math.sin(d[1])*self.scale
                py = self.MAX_RANGE*math.cos(d[1])*self.scale
            else:
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
        RTz = self.RTz(0, 0, 0, 0)
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
        if posx > 0.0:
            px1 = px + sidex * math.cos(math.pi+ang-0.5)
            py1 = py + sidey * math.sin(math.pi+ang-0.5)
            px2 = px + sidex * math.cos(math.pi+ang+0.5)
            py2 = py + sidey * math.sin(math.pi+ang+0.5)

	elif posx == 0.0:
            px1 = px + sidex * math.cos(ang-0.5)
            py1 = py + sidey * math.sin(ang-0.5)
            px2 = px + sidex * math.cos(ang+0.5)
            py2 = py + sidey * math.sin(ang+0.5)       

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
        painter.drawLine(QPointF(sy*self.scale,sx*self.scale),QPointF(ey*self.scale,ex*self.scale))
        painter.drawText( QPoint(sy*self.scale+3,sx*self.scale), self.targetid );


        sx = posx + 0.25
        sy = posy - 0.25
        ex = posx - 0.25
        ey = posy + 0.25
        painter.drawLine(QPointF(sy*self.scale,sx*self.scale),QPointF(ey*self.scale,ex*self.scale))

    def setCarArrow(self, x, y):
        self.carx = x
        self.cary = y

    def setObstaclesArrow(self, x, y):
        self.obsx = x
        self.obsy = y

    def setAverageArrow(self, x, y):
        self.avgx = x
        self.avgy = y

    def setTarget(self, x, y, rx, ry, rt, id):
        # Convert to relatives
        #self.targetx = x - rx
        #self.targety = y - ry
        if x == 0.0 and y == 0.0:
            return        

        dx = rx - x
        dy = ry - y

        # Rotate with current angle
        self.targetx = dx*math.cos(-rt) - dy*math.sin(-rt)
        self.targety = dx*math.sin(-rt) + dy*math.cos(-rt)
        self.targetid = id

    def setLaserValues(self, laser):

        # Init laser array
        self.MAX_RANGE = laser.maxRange
        angle = int(round(math.degrees(laser.maxAngle)))
        if len(self.laser) == 0:
            for i in range(angle):
                self.laser.append((0,0))

        for i in range(angle):
            dist = laser.values[i]
            angle = math.radians(i)
            self.laser[i] = (dist, angle)



 
       

