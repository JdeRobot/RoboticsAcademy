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
import resources_rc
from PyQt5 import QtGui
from PyQt5.QtCore import pyqtSignal, QPointF, Qt, QPoint
from PyQt5.QtWidgets import QWidget, QGridLayout

class LogoWidget(QWidget):

    def __init__(self,winParent):    
        super(LogoWidget, self).__init__()
        self.winParent=winParent
        self.qimage=QtGui.QImage()
        self.qimage.load(':images/jderobot.png')


    def paintEvent(self, e):

        painter=QtGui.QPainter(self)
        painter.drawImage(self.width()/2-self.qimage.width()/2, self.height()/2-self.qimage.height()/2, self.qimage)


