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
from PyQt5.QtWidgets import QGraphicsEllipseItem, QGraphicsObject, QGraphicsItem
from PyQt5.QtCore import Qt, pyqtSignal, QRectF
from PyQt5.QtGui import QBrush, QPen
from gui.idtextboxgraphicsitem import IdTextBoxGraphicsItem

class StateGraphicsItem(QGraphicsObject):
    # constant values
    NODE_WIDTH = 40
    INIT_WIDTH = 30
    PEN_NORMAL_WIDTH = 1
    PEN_FOCUS_WIDTH = 3

    posChanged = pyqtSignal('QGraphicsItem')
    stateNameChanged = pyqtSignal('QGraphicsItem')

    stateTextEditStarted = pyqtSignal()
    stateTextEditFinished = pyqtSignal()
    doubleClicked = pyqtSignal('QGraphicsItem')

    def __init__(self, data):
        super(QGraphicsObject, self).__init__()
        self.stateData = data
        self.setAcceptHoverEvents(True)
        self.setFlag(QGraphicsItem.ItemIsMovable)
        self.setAcceptDrops(True)

        # position of the graphics item on the scene
        self.setPos(self.stateData.x, self.stateData.y)

        self.dragging = False

        # create an ellipse
        self.ellipse = QGraphicsEllipseItem(-StateGraphicsItem.NODE_WIDTH / 2,
                                            -StateGraphicsItem.NODE_WIDTH / 2,
                                            StateGraphicsItem.NODE_WIDTH,
                                            StateGraphicsItem.NODE_WIDTH, self)
        brush = QBrush(Qt.SolidPattern)
        brush.setColor(Qt.blue)
        self.ellipse.setBrush(brush)

        self.textGraphics = IdTextBoxGraphicsItem(self.stateData.name, self)
        textWidth = self.textGraphics.boundingRect().width()
        self.textGraphics.setPos(-textWidth / 2, StateGraphicsItem.NODE_WIDTH -
                                 (StateGraphicsItem.NODE_WIDTH / 2) + 5)
        self.textGraphics.textChanged.connect(self.nameChanged)
        self.textGraphics.textEditStarted.connect(self.textEditStarted)
        self.textGraphics.textEditFinished.connect(self.textEditFinished)

        self.initGraphics = None
        self.setInitial(self.stateData.initial)

    def setInitial(self, initial):
        if initial:
            if self.initGraphics is None:
                self.initGraphics = QGraphicsEllipseItem(-StateGraphicsItem.INIT_WIDTH / 2,
                                                         -StateGraphicsItem.INIT_WIDTH / 2,
                                                         StateGraphicsItem.INIT_WIDTH,
                                                         StateGraphicsItem.INIT_WIDTH, self)
            else:
                self.initGraphics.setParentItem(self)
        else:
            if self.initGraphics is not None:
                self.initGraphics.setParentItem(None)

    def hoverEnterEvent(self, event):
        myPen = QPen(Qt.SolidLine)
        myPen.setWidth(StateGraphicsItem.PEN_FOCUS_WIDTH)
        self.ellipse.setPen(myPen)
        super(QGraphicsObject, self).hoverEnterEvent(event)

    def hoverLeaveEvent(self, event):
        myPen = QPen(Qt.SolidLine)
        myPen.setWidth(StateGraphicsItem.PEN_NORMAL_WIDTH)
        self.ellipse.setPen(myPen)
        super(QGraphicsObject, self).hoverLeaveEvent(event)

    def mousePressEvent(self, qGraphicsSceneMouseEvent):
        if qGraphicsSceneMouseEvent.button() == Qt.LeftButton:
            self.dragging = True
        super(QGraphicsObject, self).mousePressEvent(qGraphicsSceneMouseEvent)

    def mouseReleaseEvent(self, qGraphicsSceneMouseEvent):
        if qGraphicsSceneMouseEvent.button() == Qt.LeftButton:
            self.dragging = False
        super(QGraphicsObject, self).mouseReleaseEvent(qGraphicsSceneMouseEvent)

    def mouseMoveEvent(self, qGraphicsSceneMouseEvent):
        if self.dragging:
            self.posChanged.emit(self)
        super(QGraphicsObject, self).mouseMoveEvent(qGraphicsSceneMouseEvent)

    def mouseDoubleClickEvent(self, qGraphicsSceneMouseEvent):
        self.doubleClicked.emit(self)

    def boundingRect(self):
        return self.ellipse.boundingRect()

    def nameChanged(self, newName):
        self.stateData.name = newName
        textWidth = self.textGraphics.boundingRect().width()
        # reposition to center the text
        self.textGraphics.setPos(-textWidth / 2, StateGraphicsItem.NODE_WIDTH -
                                 (StateGraphicsItem.NODE_WIDTH / 2) + 5)
        self.stateNameChanged.emit(self)

    def textEditStarted(self):
        self.stateTextEditStarted.emit()

    def textEditFinished(self):
        self.stateTextEditFinished.emit()

    def paint(self, QPainter, QStyleOptionGraphicsItem, QWidget_widget=None):
        pass

    def setRunning(self, status):
        brush = QBrush(Qt.SolidPattern)
        if status:
            brush.setColor(Qt.green)
        else:
            brush.setColor(Qt.blue)

        self.ellipse.setBrush(brush)