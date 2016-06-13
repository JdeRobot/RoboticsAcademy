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

from resources import resources_rc
from PyQt4 import QtGui, QtCore
import math

class MapWidget(QtGui.QWidget):

    stopSIG=QtCore.pyqtSignal()
    
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
        self.scale = 30.0
        self.laser = []
        
    def initUI(self):
        layout=QtGui.QGridLayout()  
        self.setLayout(layout)
        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), QtCore.Qt.white)
        self.setPalette(p)
        self.resize(300,300)
        self.setMinimumSize(300,300)
        
    def paintEvent(self, e):
        _width = self.width()
        _height = self.height()
    
        painter=QtGui.QPainter(self)
        pen = QtGui.QPen(QtCore.Qt.blue, 2)
        painter.setPen(pen)
    
        #Widget center
        painter.translate(QtCore.QPoint(_width/2, _height/1.2))
           
        # Draw car
        self.drawCar(painter)

        # Draw laser
        self.drawLasel(painter)

        # Draw target
        self.drawTarget(painter, self.targetx, self.targety)

        # Draw arrows
        self.drawArrow(painter, self.carx, self.cary, QtCore.Qt.green, 2)
        self.drawArrow(painter, self.obsx, self.obsy, QtCore.Qt.red, 2)
        self.drawArrow(painter, self.avgx, self.avgy, QtCore.Qt.black, 2)

    def drawCar(self, painter):
        carsize = 30
        tiresize = carsize/5

        pen = QtGui.QPen(QtCore.Qt.black, 1)
        painter.setPen(pen)

        # Connectors
        painter.drawLine(QtCore.QPointF(-carsize/5,carsize/5),QtCore.QPointF(carsize/5, carsize/5))

        # Chassis
        painter.fillRect(-carsize/6,carsize/2,carsize/3,carsize/2,QtCore.Qt.red)
        painter.fillRect(-carsize/16,0,carsize/8,carsize,QtCore.Qt.red)
        painter.fillRect(-carsize/6,-carsize/24,carsize/3,carsize/12,QtCore.Qt.red)
        painter.fillRect(-carsize/8,carsize-carsize/96,carsize/4,carsize/12,QtCore.Qt.red)

        # Tires
        painter.fillRect(-carsize/4,carsize/8,tiresize/2,tiresize,QtCore.Qt.black)
        painter.fillRect(carsize/4,carsize/8,-tiresize/2,tiresize,QtCore.Qt.black)
        painter.fillRect(-carsize/4,carsize-carsize/8,tiresize/2,tiresize,QtCore.Qt.black)
        painter.fillRect(carsize/4,carsize-carsize/8,-tiresize/2,tiresize,QtCore.Qt.black)

    def drawLasel(self, painter):
        pen = QtGui.QPen(QtCore.Qt.blue, 2)
        painter.setPen(pen)
        for d in self.laser:
            px = -d[0]*math.sin(d[1])*self.scale
            py = d[0]*math.cos(d[1])*self.scale
            painter.drawLine(QtCore.QPointF(0,0),QtCore.QPointF(py, px))
            
    def drawArrow(self, painter, posx, posy, color, width):
        if posx == 0.0 and posy == 0.0:
            return        

        _width = self.width()
        _height = self.height()

        pen = QtGui.QPen(color, width)
        painter.setPen(pen)

        # Draw main line
        px = _width/2*posx/10.0
        py = _height/2*posy/10.0
        painter.drawLine(QtCore.QPointF(0,0),QtCore.QPointF(-px, py))

        # Draw sides
        sidex = math.hypot(px, py)/5.0
        sidey = math.hypot(px, py)/5.0
        if px != 0.0:
            ang = math.atan(py/px)
        else:
            ang = math.pi/2.0
        if posx >= 0.0:
            px1 = px + sidex * math.cos(math.pi+ang-0.5)
            py1 = py + sidey * math.sin(math.pi+ang-0.5)
            px2 = px + sidex * math.cos(math.pi+ang+0.5)
            py2 = py + sidey * math.sin(math.pi+ang+0.5)
        else:
            px1 = px - sidex * math.cos(math.pi+ang-0.5)
            py1 = py - sidey * math.sin(math.pi+ang-0.5)
            px2 = px - sidex * math.cos(math.pi+ang+0.5)
            py2 = py - sidey * math.sin(math.pi+ang+0.5)    
        painter.drawLine(QtCore.QPointF(-px, py),QtCore.QPointF(-px1, py1))
        painter.drawLine(QtCore.QPointF(-px, py),QtCore.QPointF(-px2, py2))

    def drawTarget(self, painter, posx, posy):
        pen = QtGui.QPen(QtCore.Qt.red, 4)
        painter.setPen(pen)

        sx = posx - 0.25
        sy = posy - 0.25
        ex = posx + 0.25
        ey = posy + 0.25
        painter.drawLine(QtCore.QPointF(-sx*self.scale,sy*self.scale),QtCore.QPointF(-ex*self.scale,ey*self.scale))

        sx = posx + 0.25
        sy = posy - 0.25
        ex = posx - 0.25
        ey = posy + 0.25
        painter.drawLine(QtCore.QPointF(-sx*self.scale,sy*self.scale),QtCore.QPointF(-ex*self.scale,ey*self.scale))

    def setCarArrow(self, x, y):
        self.carx = x
        self.cary = y

    def setObstaclesArrow(self, x, y):
        self.obsx = x
        self.obsy = y

    def setAverageArrow(self, x, y):
        self.avgx = x
        self.avgy = y

    def setTarget(self, x, y, rx, ry, rt):
        # Convert to relatives
        dx = x - rx
        dy = y - ry

        # Rotate with current angle
        self.targetx = dx*math.cos(-rt) - dy*math.sin(-rt)
        self.targety = dx*math.sin(-rt) + dy*math.cos(-rt)

    def setLaserValues(self, laser):
        # Init laser array
        if len(self.laser) == 0:
            for i in range(laser.numLaser):
                self.laser.append((0,0))

        for i in range(laser.numLaser):
            dist = laser.distanceData[i]/1000.0
            angle = math.radians(i)
            self.laser[i] = (dist, angle)



 
       

