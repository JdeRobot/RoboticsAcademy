#  Authors :
#       Samuel Rey Escudero <samuel.rey.escudero@gmail.com>
#

import sys
import threading
from PyQt4 import QtGui, QtCore
import cv2

class Map(QtGui.QWidget):
    
    MAP1 = "resources/images/cityLarge.png"
    MAP2 = "resources/images/cityMedium.png"

    def __init__(self, winParent):
        super(Map, self).__init__()
        self.lock = threading.Lock()
        self.parent = winParent        

        self.initUI()

        self.lastPos = None
        self.lastDest = None
        self.lastPath = None


    def initUI(self):
        self.map = cv2.imread(self.MAP1)
        image = QtGui.QImage(self.map.data, self.map.shape[1], self.map.shape[0], self.map.shape[1]*self.map.shape[2], QtGui.QImage.Format_RGB888);
        self.pixmap = QtGui.QPixmap.fromImage(image)
        self.height = self.pixmap.height()
        self.width = self.pixmap.width()
        self.mapWidget = QtGui.QLabel(self)
        self.mapWidget.setPixmap(self.pixmap)
        self.mapWidget.resize(self.width, self.height)


    def mouseDoubleClickEvent(self, event):
        x = event.pos().x()
        y = event.pos().y()
        print "Destiny: ", x, ", ", y
        self.parent.grid.setDestiny(x, y)


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
        #Compensating the position
        x = x - 4

        triangle = QtGui.QPolygon()
        triangle.append(QtCore.QPoint(x+4, y-4))
        triangle.append(QtCore.QPoint(x+4, y+4))
        triangle.append(QtCore.QPoint(x-5, y))
        matrix = QtGui.QMatrix()
        matrix.rotate(-angle)
        triangle = matrix.map(triangle)
        center = matrix.map(QtCore.QPoint(x, y))
        xDif = x - center.x()
        yDif = y - center.y()
        triangle.translate(xDif, yDif)

        self.setPainterSettings(painter, QtCore.Qt.yellow, 1)
        painter.drawPolygon(triangle)

    
    def paintDestiny(self, painter, dest):
        painter.drawLine(dest[0]-3, dest[1]+3, dest[0]+3, dest[1]-3)
        painter.drawLine(dest[0]+3, dest[1]+3, dest[0]-3, dest[1]-3)


    def paintPath(self, painter, path):
        points = QtGui.QPolygonF()
        for i in range(path.shape[0]):
            for j in range(path.shape[1]):
                if path[i][j] == 1:
                    points.append(QtCore.QPointF(j, i))

        self.setPainterSettings(painter, QtCore.Qt.green, 2)
        painter.drawPoints(points)



    def isNewPos(self, pos):
        return (self.lastPos == None or (self.lastPos[0] != pos[0] and self.lastPos[1] != pos[1]))


    def updateMap(self, grid):
        pos = grid.getPose()
        dest = grid.getDestiny()
        path = grid.getPath()

        if (pos == self.lastPos and dest == self.lastDest):
            if (path != None and self.lastPath != None):
                if (path.all() == self.lastPath.all()):
                    return                

        self.lock.acquire()
        copy = self.pixmap.copy()
        painter = self.getPainter(copy)

        if dest != None:
            self.setPainterSettings(painter, QtCore.Qt.red, 3)
            self.paintDestiny(painter, dest)

        if path != None:
            self.setPainterSettings(painter, QtCore.Qt.green, 3)
            self.paintPath(painter, path)

        self.paintPosition(pos[0], pos[1], grid.getAngle(), copy, painter)

        self.mapWidget.setPixmap(copy)
        self.lock.release()
        painter.end()

        self.lastPos = pos
        self.lastDest = dest
        self.lastPath = path