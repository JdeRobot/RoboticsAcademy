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
#       Francisco Miguel Rivas Montero <franciscomiguel.rivas@urjc.es>
#

from gui.widgets.teleopWidget import TeleopWidget
from gui.widgets.mapWidget import MapWidget
from PyQt4 import QtGui,QtCore
from gui.form import Ui_MainWindow
from gui.widgets.cameraWidget import CameraWidget

class MainWindow(QtGui.QMainWindow, Ui_MainWindow):

    updGUI=QtCore.pyqtSignal()
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.teleop=TeleopWidget(self)
        self.map=MapWidget(self)
        self.tlLayout.addWidget(self.teleop)
        self.teleop.setVisible(True)
        self.mapLayout.addWidget(self.map)
        self.map.setVisible(True)

        self.pushButton.clicked.connect(self.playClicked)
        self.pushButton.setCheckable(True)
        self.updGUI.connect(self.updateGUI)
        self.camera1=CameraWidget(self)

        self.stopButton.clicked.connect(self.stopClicked)

    def updateGUI(self):
        self.camera1.updateImage()
        (cx, cy) = self.algorithm.getCarDirection()
        (ox, oy) = self.algorithm.getObstaclesDirection()
        (ax, ay) = self.algorithm.getAverageDirection()
        (tx, ty) = self.algorithm.getCurrentTarget()
        self.map.setCarArrow(cx, cy)
        self.map.setObstaclesArrow(ox, oy)
        self.map.setAverageArrow(ax, ay)
        self.map.setTarget(tx, ty, self.sensor.getRobotX(), self.sensor.getRobotY(), self.sensor.getRobotTheta())
        self.map.setLaserValues(self.sensor.getLaserData())
        self.map.update()

    def getSensor(self):
        return self.sensor

    def setSensor(self,sensor):
        self.sensor=sensor

    def playClicked(self):
        self.sensor.setPlayButton(self.pushButton.isChecked())
        if self.pushButton.isChecked():
            self.pushButton.setText('RUNNING')
            self.pushButton.setStyleSheet("background-color: green")
        else:
            self.pushButton.setText('STOPPED')
            self.pushButton.setStyleSheet("background-color: red")

    def setAlgorithm(self, algorithm ):
        self.algorithm=algorithm

    def getAlgorithm(self):
        return self.algorithm

    def setXYValues(self,newX,newY):
        self.sensor.setV(-newY)
        self.sensor.setW(newX)

    def stopClicked(self):
        self.sensor.setV(0)
        self.sensor.setW(0)
        self.teleop.returnToOrigin()
