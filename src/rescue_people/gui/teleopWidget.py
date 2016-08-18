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
from resources import resources_rc
from PyQt4 import QtGui, QtCore

class TeleopWidget(QtGui.QWidget):

    stopSIG=QtCore.pyqtSignal()
    
    def __init__(self,winParent):    
        super(TeleopWidget, self).__init__()
        self.winParent=winParent
        self.line = QtCore.QPointF(0, 0);
        self.qimage=QtGui.QImage()
        self.qimage.load(':images/ball.png')
        self.stopSIG.connect(self.stop)
        self.initUI()
        
    def initUI(self):
        layout=QtGui.QGridLayout()  
        self.setLayout(layout)
        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), QtCore.Qt.black)
        self.setPalette(p)
        self.resize(300,300)
        self.setMinimumSize(300,300)
        
    def stop(self):
        self.line = QtCore.QPointF(0, 0);
        self.repaint();
    
    def mouseMoveEvent(self,e):
        if e.buttons() == QtCore.Qt.LeftButton:
            x = e.x()-self.width()/2
            y = e.y()-self.height()/2
            self.line = QtCore.QPointF(x, y)
            self.repaint()

    def paintEvent(self, e):
        _width = self.width()
        _height = self.height()
    
    
        width = 2
    
        painter=QtGui.QPainter(self)
    
        pen = QtGui.QPen(QtCore.Qt.blue, width)
        painter.setPen(pen)
    
        #Centro del widget
        painter.translate(QtCore.QPoint(_width/2, _height/2))
    
        #eje
        painter.drawLine(QtCore.QPointF(-_width, 0),
                QtCore.QPointF( _width, 0))
    
        painter.drawLine(QtCore.QPointF(0, -_height),
                QtCore.QPointF(0, _height))
    
        #con el raton
        pen = QtGui.QPen(QtCore.Qt.red, width)
        painter.setPen(pen)

        #Comprobamos que el raton este dentro de los limites
        if abs(self.line.x()*2) >= self.size().width():
            if self.line.x()>=0:
                self.line.setX(self.size().width()/2)
            elif self.line.x()<0:	
                self.line.setX((-self.size().width()/2)+1)
        
        if abs(self.line.y()*2) >= self.size().height():
            if self.line.y()>=0:
                self.line.setY(self.size().height()/2)
            elif self.line.y()<0:	
                self.line.setY((-self.size().height()/2)+1)

        painter.drawLine(QtCore.QPointF(self.line.x(), -_height),
                QtCore.QPointF(self.line.x(), _height))
    
        painter.drawLine(QtCore.QPointF(-_width, self.line.y()),
                QtCore.QPointF( _width, self.line.y()))

        #print "x: %f y: %f" % (self.line.x(), self.line.y())

        v_normalized = (1.0/(self.size().height()/2)) * self.line.y()
        v_normalized = float("{0:.2f}".format(v_normalized))
        w_normalized = (1.0/(self.size().width()/2)) * self.line.x()
        w_normalized = float("{0:.2f}".format(w_normalized))

        #print "v: %f w: %f" % (v_normalized,w_normalized)
        self.winParent.setXYValues(w_normalized,v_normalized)
        painter.drawImage(self.line.x()-self.qimage.width()/2, self.line.y()-self.qimage.height()/2, self.qimage);

