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
import math
from PyQt4 import Qt
import PyQt4.Qwt5 as Qwt

def enumList(enum, sentinel):
    '''
    '''
    return [enum(i) for i in range(sentinel)]

colorGroupList = enumList(
    Qt.QPalette.ColorGroup, Qt.QPalette.NColorGroups)
colorRoleList = enumList(
    Qt.QPalette.ColorRole, Qt.QPalette.NColorRoles)
handList  = enumList(
    Qwt.QwtAnalogClock.Hand, Qwt.QwtAnalogClock.NHands)

class AttitudeIndicatorNeedle(Qwt.QwtDialNeedle):

    def __init__(self, color):
        Qwt.QwtDialNeedle.__init__(self)
        palette = Qt.QPalette()
        for colourGroup in colorGroupList:
            palette.setColor(colourGroup, Qt.QPalette.Text, color)
        self.setPalette(palette)

    # __init__()
    
    def draw(self, painter, center, length, direction, cg):
        direction *= math.pi / 180.0
        triangleSize = int(round(length * 0.1))

        painter.save()

        p0 = Qt.QPoint(center.x() + 1, center.y() + 1)
        p1 = Qwt.qwtPolar2Pos(p0, length - 2 * triangleSize - 2, direction)

        pa = Qt.QPolygon([
            Qwt.qwtPolar2Pos(p1, 2 * triangleSize, direction),
            Qwt.qwtPolar2Pos(p1, triangleSize, direction + math.pi/2),
            Qwt.qwtPolar2Pos(p1, triangleSize, direction - math.pi/2),
            ])

        color = self.palette().color(cg, Qt.QPalette.Text)
        painter.setBrush(color)
        painter.drawPolygon(pa)

        painter.setPen(Qt.QPen(color, 3))
        painter.drawLine(
            Qwt.qwtPolar2Pos(p0, length - 2, direction + math.pi/2),
            Qwt.qwtPolar2Pos(p0, length - 2, direction - math.pi/2))

        painter.restore()

    # draw()

# class AttitudeIndicatorNeedle


class AttitudeIndicator(Qwt.QwtDial):

    def __init__(self, *args):
        Qwt.QwtDial.__init__(self, *args)
        self.__gradient = 0.0
        self.setMode(Qwt.QwtDial.RotateScale)
        self.setWrapping(True)
        self.setOrigin(270.0)
        self.setScaleOptions(Qwt.QwtDial.ScaleTicks)
        self.setScale(0, 0, 30.0)
        self.setNeedle(AttitudeIndicatorNeedle(
            self.palette().color(Qt.QPalette.Text)))

    # __init__()

    def angle(self):
        return self.value()

    # angle()
    
    def setAngle(self, angle):
        self.setValue(angle)

    # setAngle()

    def gradient(self):
        return self.__gradient

    # gradient()

    def setGradient(self, gradient):
        self.__gradient = gradient

    # setGradient()
    
    def keyPressEvent(self, event):
        if event.key() == Qt.Qt.Key_Plus:
            self.setGradient(self.gradient() + 0.05)
        elif event.key() == Qt.Qt.Key_Minus:
            self.setGradient(self.gradient() - 0.05)
        else:
            Qwt.QwtDial.keyPressEvent(self, event)

    # keyPressEvent()

    def drawScale(self, painter, center, radius, origin, minArc, maxArc):
        dir = (360.0 - origin) * math.pi / 180.0
        offset = 4
        p0 = Qwt.qwtPolar2Pos(center, offset, dir + math.pi)

        w = self.contentsRect().width()

        # clip region to swallow 180 - 360 degrees
        pa = []
        pa.append(Qwt.qwtPolar2Pos(p0, w, dir - math.pi/2))
        pa.append(Qwt.qwtPolar2Pos(pa[-1], 2 * w, dir + math.pi/2))
        pa.append(Qwt.qwtPolar2Pos(pa[-1], w, dir))
        pa.append(Qwt.qwtPolar2Pos(pa[-1], 2 * w, dir - math.pi/2))

        painter.save()
        painter.setClipRegion(Qt.QRegion(Qt.QPolygon(pa)))
        Qwt.QwtDial.drawScale(
            self, painter, center, radius, origin, minArc, maxArc)
        painter.restore()

    # drawScale()
    
    def drawScaleContents(self, painter, center, radius):
        dir = 360 - int(round(self.origin() - self.value()))
        arc = 90 + int(round(self.gradient() * 90))
        skyColor = Qt.QColor(38, 151, 221)
        painter.save()
        painter.setBrush(skyColor)
        painter.drawChord(
            self.scaleContentsRect(), (dir - arc)*16, 2*arc*16)
        painter.restore()

    # drawScaleContents()

# class AttitudeIndicator