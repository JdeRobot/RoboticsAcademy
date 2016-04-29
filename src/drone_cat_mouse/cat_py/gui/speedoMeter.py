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
from PyQt4 import Qt
import PyQt4.Qwt5 as Qwt

class SpeedoMeter(Qwt.QwtDial):

    def __init__(self, *args):
        Qwt.QwtDial.__init__(self, *args)
        self.__label = 'km/h'
        self.setWrapping(False)
        self.setReadOnly(True)

        self.setOrigin(135.0)
        self.setScaleArc(0.0, 270.0)
        
        self.setNeedle(Qwt.QwtDialSimpleNeedle(
            Qwt.QwtDialSimpleNeedle.Arrow,
            True,
            Qt.QColor(Qt.Qt.red),
            Qt.QColor(Qt.Qt.gray).light(130)))

        self.setScaleOptions(Qwt.QwtDial.ScaleTicks | Qwt.QwtDial.ScaleLabel)
        self.setScaleTicks(0, 4, 8)

    # __init__()
    
    def setLabel(self, text):
        self.__label = text
        self.update()

    # setLabel()
    
    def label(self):
        return self.__label

    # label()
    
    def drawScaleContents(self, painter, center, radius):
        rect = Qt.QRect(0, 0, 2 * radius, 2 * radius - 10)
        rect.moveCenter(center)
        painter.setPen(self.palette().color(Qt.QPalette.Text))
        painter.drawText(
            rect, Qt.Qt.AlignBottom | Qt.Qt.AlignHCenter, self.__label)

    # drawScaleContents

# class SpeedoMeter