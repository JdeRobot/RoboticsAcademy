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
from PyQt4 import QtGui
from PyQt4.QtCore import Qt

class LogoWidget(QtGui.QWidget):

    def __init__(self,winParent, width=0, height=0):    
        super(LogoWidget, self).__init__()
        self.winParent=winParent
        self.qimage=QtGui.QImage()
        self.qimage.load(':images/jderobot.svg')
        if (width != 0 and height != 0):
        	self.qimage = self.qimage.scaled(0.8*width, 0.8*height, Qt.KeepAspectRatio)
        	self.resize(width, height)
        else:
        	self.qimage = qimage


    def paintEvent(self, e):

        painter=QtGui.QPainter(self)
        painter.drawImage(self.width()/2-self.qimage.width()/2, self.height()/2-self.qimage.height()/2, self.qimage)
