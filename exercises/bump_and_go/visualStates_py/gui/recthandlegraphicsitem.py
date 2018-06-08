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
from PyQt5.QtWidgets import QGraphicsRectItem
from PyQt5.QtGui import QPen, QBrush, QPolygonF
from PyQt5.QtCore import Qt

SQUARE_SIDE = 10
PEN_FOCUS_WIDTH = 3
PEN_NORMAL_WIDTH = 1

class RectHandleGraphicsItem(QGraphicsRectItem):
    def __init__(self, width, parent=None):
        super(QGraphicsRectItem, self).__init__(-SQUARE_SIDE / 2, -SQUARE_SIDE / 2, SQUARE_SIDE, SQUARE_SIDE, parent)
        self.setAcceptHoverEvents(True)

        # set the color of the rectangle
        brush = QBrush(Qt.SolidPattern)
        brush.setColor(Qt.red)
        self.setBrush(brush)

        self.dragging = False
        self.interaction = True

    def hoverEnterEvent(self, event):
        if self.interaction:
            myPen = QPen(Qt.SolidLine)
            myPen.setWidth(PEN_FOCUS_WIDTH)
            self.setPen(myPen)

    def hoverLeaveEvent(self, event):
        if self.interaction:
            myPen = QPen(Qt.SolidLine)
            myPen.setWidth(PEN_NORMAL_WIDTH)
            self.setPen(myPen)

    def mousePressEvent(self, qGraphicsSceneMouseEvent):
        if qGraphicsSceneMouseEvent.button() == Qt.LeftButton:
            self.dragging = True
        super(QGraphicsRectItem, self).mousePressEvent(qGraphicsSceneMouseEvent)

    def mouseReleaseEvent(self, qGraphicsSceneMouseEvent):
        if qGraphicsSceneMouseEvent.button() == Qt.LeftButton:
            self.dragging = False
        super(QGraphicsRectItem, self).mouseReleaseEvent(qGraphicsSceneMouseEvent)

    def mouseMoveEvent(self, qGraphicsSceneMouseEvent):
        if self.dragging:
            self.parentItem().updateMiddlePoints(self.scenePos())
        super(QGraphicsRectItem, self).mouseMoveEvent(qGraphicsSceneMouseEvent)

    def disableInteraction(self):
        self.interaction = False