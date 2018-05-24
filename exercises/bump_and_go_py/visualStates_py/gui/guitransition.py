'''
   Copyright (C) 1997-2017 JDERobot Developers Team

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Library General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, see <http://www.gnu.org/licenses/>.

   Authors : Okan Asik (asik.okan@gmail.com)

  '''
from PyQt5.QtWidgets import QGraphicsObject, QGraphicsLineItem, QGraphicsPolygonItem, QGraphicsItem
from PyQt5.QtGui import QBrush, QPolygonF
from PyQt5.QtCore import Qt, QPointF, QLineF, pyqtSignal

from gui.guistate import StateGraphicsItem
from gui.idtextboxgraphicsitem import IdTextBoxGraphicsItem
from gui.recthandlegraphicsitem import RectHandleGraphicsItem
import math

class TransitionGraphicsItem(QGraphicsObject):
    # constant values
    SQUARE_SIDE = 10
    ARROW_SIZE = 12
    PEN_NORMAL_WIDTH = 1
    PEN_FOCUS_WIDTH = 3

    posChanged = pyqtSignal('QGraphicsItem')

    def __init__(self, data):
        super(QGraphicsObject, self).__init__()
        self.transitionData = data

        self.originLine = None
        self.destinationLine = None
        self.arrow = None
        self.textGraphics = None
        self.middleHandle = None

        self.graphicsOrigin = self.transitionData.origin.getGraphicsItem()
        self.graphicsDestination = self.transitionData.destination.getGraphicsItem()

        # connect position changed event
        self.graphicsOrigin.posChanged.connect(self.statePosChanged)
        self.graphicsDestination.posChanged.connect(self.statePosChanged)

        self.midPointX = (self.graphicsDestination.scenePos().x() + self.graphicsOrigin.scenePos().x()) / 2.0
        self.midPointY = (self.graphicsDestination.scenePos().y() + self.graphicsOrigin.scenePos().y()) / 2.0

        self.createOriginLine()
        self.createDestinationLine()

        self.createArrow()
        self.createMiddleHandle()
        self.createIdTextBox()

    def statePosChanged(self, state):
        if self.graphicsOrigin == state:
            self.createOriginLine()
        elif self.graphicsDestination == state:
            self.createDestinationLine()
            self.createArrow()

    def createOriginLine(self):
        if self.originLine == None:
            self.originLine = QGraphicsLineItem(self.midPointX, self.midPointY, self.graphicsOrigin.scenePos().x(),
                                                self.graphicsOrigin.scenePos().y(), self)
        else:
            self.originLine.setLine(QLineF(self.midPointX, self.midPointY, self.graphicsOrigin.scenePos().x(),
                                           self.graphicsOrigin.scenePos().y()))
        myLine = self.originLine.line()
        myLine.setLength(myLine.length() - StateGraphicsItem.NODE_WIDTH / 2)
        self.originLine.setLine(myLine)

    def createDestinationLine(self):
        if self.destinationLine == None:
            self.destinationLine = QGraphicsLineItem(self.midPointX, self.midPointY, self.graphicsDestination.scenePos().x(),
                                                     self.graphicsDestination.scenePos().y(), self)
        else:
            self.destinationLine.setLine(QLineF(self.midPointX, self.midPointY, self.graphicsDestination.scenePos().x(),
                                                self.graphicsDestination.scenePos().y()))

        myLine = self.destinationLine.line()
        myLine.setLength(myLine.length() - StateGraphicsItem.NODE_WIDTH / 2)
        self.destinationLine.setLine(myLine)

    def createArrow(self):
        # add an arrow to destination line
        myLine = self.destinationLine.line()
        myLine.setLength(myLine.length() - TransitionGraphicsItem.ARROW_SIZE)
        rotatePoint = myLine.p2() - self.destinationLine.line().p2()

        rightPointX = rotatePoint.x() * math.cos(math.pi / 6) - rotatePoint.y() * math.sin(math.pi / 6)
        rightPointY = rotatePoint.x() * math.sin(math.pi / 6) + rotatePoint.y() * math.cos(math.pi / 6)
        rightPoint = QPointF(rightPointX + self.destinationLine.line().x2(),
                             rightPointY + self.destinationLine.line().y2())

        leftPointX = rotatePoint.x() * math.cos(-math.pi / 6) - rotatePoint.y() * math.sin(-math.pi / 6)
        leftPointY = rotatePoint.x() * math.sin(-math.pi / 6) + rotatePoint.y() * math.cos(-math.pi / 6)
        leftPoint = QPointF(leftPointX + self.destinationLine.line().x2(),
                            leftPointY + self.destinationLine.line().y2())

        polygon = QPolygonF()
        polygon << rightPoint << leftPoint << self.destinationLine.line().p2() << rightPoint

        if self.arrow == None:
            self.arrow = QGraphicsPolygonItem(polygon, self)
        else:
            self.arrow.setPolygon(polygon)

        brush = QBrush(Qt.SolidPattern)
        brush.setColor(Qt.black)
        self.arrow.setBrush(brush)

    def createMiddleHandle(self):
        # create middle handle
        if self.middleHandle == None:
            self.middleHandle = RectHandleGraphicsItem(TransitionGraphicsItem.SQUARE_SIDE, self)
            self.middleHandle.setFlag(QGraphicsItem.ItemIsMovable)

        self.middleHandle.setPos(self.midPointX, self.midPointY)

    def createIdTextBox(self):
        if self.textGraphics == None:
            self.textGraphics = IdTextBoxGraphicsItem(self.transitionData.name, self)
            self.textGraphics.textChanged.connect(self.nameChanged)
        else:
            self.textGraphics.setPlainText(self.transitionData.name)
        textWidth = self.textGraphics.boundingRect().width()
        self.textGraphics.setPos(self.midPointX - textWidth / 2, self.midPointY + TransitionGraphicsItem.SQUARE_SIDE -
                                 (TransitionGraphicsItem.SQUARE_SIDE / 2) + 5)

    def updateMiddlePoints(self, newPosition):
        self.midPointX = newPosition.x()
        self.midPointY = newPosition.y()
        self.createOriginLine()
        self.createDestinationLine()
        self.createArrow()
        self.createIdTextBox()
        self.posChanged.emit(self)

    def nameChanged(self, name):
        self.transitionData.name = name
        self.createIdTextBox()

    def boundingRect(self):
        if self.middleHandle != None:
            return self.middleHandle.boundingRect()
        else:
            return None

    def disableInteraction(self):
        if self.middleHandle is not None:
            self.middleHandle.setFlag(QGraphicsItem.ItemIsMovable, False)
            self.middleHandle.disableInteraction()