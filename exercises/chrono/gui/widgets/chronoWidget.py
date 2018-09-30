#
#  Copyright (C) 1997-2016 JDE Developers Team
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
#       Eduardo Perdices <eperdices@gsyc.es>
#

import sys
from PyQt5.QtWidgets import QWidget, QGridLayout, QLabel, QApplication
from PyQt5 import QtCore, QtGui
from PyQt5.QtCore import QPoint, QPointF, pyqtSignal, Qt, QTime, QTimer
import cv2
import rospy

class ChronoWidget(QWidget):
    def __init__(self,winParent):
        super(ChronoWidget, self).__init__()
        self.winParent=winParent
        self.labelChrono = winParent.chrono_1
        self.labelDuration = winParent.chrono_2

    def setTime(self, initime, duration):
        if initime == 0.0:
            self.labelChrono.setText("0:00 min")
        else:
            time = rospy.Time.from_sec(rospy.get_time()).to_sec() - initime
            minutes = int(time) / 60
            seconds = int(time) % 60
            if seconds < 10:
                self.labelChrono.setText(str(minutes) + ":0" + str(seconds) + " min")
            else:
                self.labelChrono.setText(str(minutes) + ":" + str(seconds) + " min")
        self.labelDuration.setText(str(duration))
