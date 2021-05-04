#  Authors :
#       Samuel Rey Escudero <samuel.rey.escudero@gmail.com>
#

import sys, math
import threading
from PyQt5 import QtGui, QtCore
from PyQt5.QtWidgets import QWidget, QLabel
from PyQt5.QtCore import QPointF
import cv2

class Map(QWidget):
    
    def __init__(self, winParent):
        super(Map, self).__init__()
        self.lock = threading.Lock()
        self.readConfFile()
        self.parent = winParent        

        self.initUI()

        self.lastPos = None
        self.lastDest = None


    def readConfFile(self):
        lines = None
        for arg in sys.argv:
            splitedArg = arg.split(".")
            if (splitedArg[1] == "conf"):
                lines = open(arg, "r").readlines()

        if not lines:
            raise Exception("Could not read map config file")

        for line in lines:
            lineSplit = line.split("=")
            if (lineSplit[0] == "img"):
                if (lineSplit[1][-1] == "\n"):
                    self.mapPath = lineSplit[1][:-1]
                else:
                    self.mapPath = lineSplit[1]
            elif (lineSplit[0] == "worldWidth"):
                self.worldWidth = int(lineSplit[1])
            elif (lineSplit[0] == "worldHeight"):
                self.worldHeight = lineSplit[1]
            elif (lineSplit[0] == "originX"):
                self.originX = int(lineSplit[1])
            elif (lineSplit[0] == "originY"):
                self.originY = int(lineSplit[1])
            elif (lineSplit[0] == "angle"):
                self.mapAngle = int(lineSplit[1]) % 360
                print("Grados:", self.mapAngle)


    def initUI(self):
        self.map = cv2.imread(self.mapPath, cv2.IMREAD_GRAYSCALE)
        print(self.map.shape)
        self.map = cv2.resize(self.map, (400, 400))
        image = QtGui.QImage(self.map.data, self.map.shape[1], self.map.shape[0], self.map.shape[1], QtGui.QImage.Format_Indexed8);
        self.pixmap = QtGui.QPixmap.fromImage(image)
        self.height = self.pixmap.height()
        self.width = self.pixmap.width()
        self.mapWidget = QLabel(self)
        self.mapWidget.setPixmap(self.pixmap)
        self.mapWidget.resize(self.width, self.height)


    def mouseDoubleClickEvent(self, event):
        x = event.pos().x()
        y = event.pos().y()
        print("Destiny: ", x, ", ", y)
        rX, rY = self.parent.grid.gridToWorld(x,y) 
        print("WORLD: ", rX, ", ", rY)
        self.parent.grid.setDestiny(x, y)
        self.parent.grid.resetPath()
        self.parent.grid.resetGrid()


    def setPainterSettings(self, painter, color, width):
        pen = QtGui.QPen(color)
        pen.setWidth(width)
        brush = QtGui.QBrush(QtCore.Qt.SolidPattern)
        brush.setColor(QtGui.QColor(color))
        painter.setPen(pen)
        painter.setBrush(brush)


    def getPainter(self, copy):
        painter = QtGui.QPainter(copy)
        return painter


    def paintPosition(self, x, y, angle, img, painter):
        triangle = QtGui.QPolygon()
        triangle.append(QtCore.QPoint(x-4, y-4))
        triangle.append(QtCore.QPoint(x+4, y-4))
        triangle.append(QtCore.QPoint(x, y+5))
        matrix = QtGui.QTransform()
        matrix.rotate(-angle + self.mapAngle)
        triangle = matrix.map(triangle)
        center = matrix.map(QtCore.QPoint(x, y))
        xDif = x - center.x()
        yDif = y - center.y()
        triangle.translate(xDif, yDif)

        self.setPainterSettings(painter, QtCore.Qt.blue, 1)
        painter.drawPolygon(triangle)

    
    def paintDestiny(self, painter, dest):
        painter.drawLine(dest[0]-3, dest[1]+3, dest[0]+3, dest[1]-3)
        painter.drawLine(dest[0]+3, dest[1]+3, dest[0]-3, dest[1]-3)


    def paintPath(self, painter, path):
        points = QtGui.QPolygonF()
        for i in range(path.shape[0]):
            for j in range(path.shape[1]):
                if path[i][j] > 0:
                    points.append(QtCore.QPointF(j, i))

        self.setPainterSettings(painter, QtCore.Qt.green, 2)
        painter.drawPoints(points)



    def isNewPos(self, pos):
        return (self.lastPos == None or (self.lastPos[0] != pos[0] and self.lastPos[1] != pos[1]))


    def updateMap(self, grid):
        pos = grid.getPose()
        dest = grid.getDestiny()
        path = grid.getPath()

        if (pos == self.lastPos and dest == self.lastDest and not grid.pathFinded):
            return              

        self.lock.acquire()
        copy = self.pixmap.copy()
        painter = self.getPainter(copy)

        if dest != None:
            self.setPainterSettings(painter, QtCore.Qt.red, 3)
            self.paintDestiny(painter, dest)

        self.setPainterSettings(painter, QtCore.Qt.green, 3)
        self.paintPath(painter, path)


        self.paintPosition(pos[0], pos[1], grid.getAngle(), copy, painter)

        self.mapWidget.setPixmap(copy)
        self.lock.release()
        painter.end()

        self.lastPos = pos
        self.lastDest = dest
        grid.pathFinded = False
