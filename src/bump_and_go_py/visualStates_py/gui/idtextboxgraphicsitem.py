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
from PyQt5.QtWidgets import QGraphicsTextItem
from PyQt5.QtCore import Qt, pyqtSignal

class IdTextBoxGraphicsItem(QGraphicsTextItem):

    textChanged = pyqtSignal('QString')
    textEditStarted = pyqtSignal()
    textEditFinished = pyqtSignal()

    def __init__(self, name, parent=None):
        super(QGraphicsTextItem, self).__init__(name, parent)
        self.name = name

    def mouseDoubleClickEvent(self, event):
        if self.textInteractionFlags() == Qt.NoTextInteraction:
            self.setTextInteractionFlags(Qt.TextEditorInteraction)
            self.textEditStarted.emit()

        QGraphicsTextItem.mouseDoubleClickEvent(self, event)


    def focusOutEvent(self, event):
        self.setTextInteractionFlags(Qt.NoTextInteraction)
        self.textChanged.emit(self.toPlainText())
        self.textEditFinished.emit()
        QGraphicsTextItem.focusOutEvent(self, event)
