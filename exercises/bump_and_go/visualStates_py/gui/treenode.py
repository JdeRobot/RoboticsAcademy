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
from PyQt5.QtWidgets import QTreeWidgetItem
from PyQt5.QtGui import QColor

class TreeNode(QTreeWidgetItem):
    def __init__(self, id, name, color, parent=None):
        super(QTreeWidgetItem, self).__init__(parent)

        self.parentItem = parent
        self.id = id
        self.name = name
        self.color = color
        self.childItems = []

    def background(self):
        background = QColor(self.color)
        return background

    def appendChild(self, item):
        self.childItems.append(item)

    def removeChild(self, item):
        self.childItems.remove(item)

    def child(self, row):
        return self.childItems[row]

    def childCount(self):
        return len(self.childItems)

    def columnCount(self):
        return 2

    def data(self, column):
        if column == 0:
            return self.id
        if column == 1:
            return self.name

    def parent(self):
        return self.parentItem

    def row(self):
        if self.parentItem:
            return self.parentItem.childItems.index(self)
        return 0

    def getChildren(self):
        return self.childItems

    def setColor(self, color):
        self.color = color

    def removeChildren(self):
        del self.childItems[:]

    def myClicked(self, item, column):
        # print('clicked:' + str(item))
        pass