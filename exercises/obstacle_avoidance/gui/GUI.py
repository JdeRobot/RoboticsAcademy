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
from PyQt5.QtCore import pyqtSignal
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QMainWindow
from gui.form import Ui_MainWindow
from gui.widgets.logoWidget import LogoWidget
from gui.widgets.cameraWidget import CameraWidget

class MainWindow(QMainWindow, Ui_MainWindow):

    updGUI=pyqtSignal()
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.teleop=TeleopWidget(self)
        self.map=MapWidget(self)
        self.tlLayout.addWidget(self.teleop)
        self.teleop.setVisible(True)
        self.mapLayout.addWidget(self.map)
        self.map.setVisible(True)


        self.verticalLayout_2.addWidget(self.stopButton,3)

        self.logo = LogoWidget(self, 60, 60)
        self.verticalLayout_2.addWidget(self.logo,4)
        self.logo.setVisible(True)

        self.playButton.clicked.connect(self.playClicked)
        self.playButton.setCheckable(True)
        self.updGUI.connect(self.updateGUI)
        #self.camera1=CameraWidget(self)

        self.stopButton.clicked.connect(self.stopClicked)

    def updateGUI(self):
        #self.camera1.updateImage()
        (cx, cy) = self.algorithm.getCarDirection()
        (ox, oy) = self.algorithm.getObstaclesDirection()
        (ax, ay) = self.algorithm.getAverageDirection()
        (tx, ty, id) = self.algorithm.getCurrentTarget()
        self.map.setCarArrow(cx, cy)
        self.map.setObstaclesArrow(ox, oy)
        self.map.setAverageArrow(ax, ay)
        if (self.pose3d):
            self.map.setTarget(tx, ty, self.pose3d.getPose3d().x, self.pose3d.getPose3d().y, self.pose3d.getPose3d().yaw, id)
        laserdata = self.laser.getLaserData()
        if (laserdata):
            self.map.setLaserValues(laserdata)
        self.map.update()

    #def getCamera(self):
        #return self.camera

    #def setCamera(self,camera):
        #self.camera=camera

    def getPose3D(self):
        return self.pose3d

    def setPose3D(self,pose3d):
        self.pose3d=pose3d

    def getLaser(self):
        return self.laser

    def setLaser(self,laser):
        self.laser=laser

    def getMotors(self):
        return self.motors

    def setMotors(self,motors):
        self.motors=motors

    def playClicked(self):
        if self.playButton.isChecked():
            icon = QtGui.QIcon()
            self.playButton.setStyleSheet("background-color: #ec7063")
            icon.addPixmap(QtGui.QPixmap(":/images/stop.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.playButton.setIcon(icon)
            self.algorithm.play()
        else:
            icon = QtGui.QIcon()
            self.playButton.setStyleSheet("background-color: #7dcea0")
            icon.addPixmap(QtGui.QPixmap(":/images/play.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.playButton.setIcon(icon)
            self.algorithm.stop()
            self.motors.sendV(0)
            self.motors.sendW(0)
            #self.motors.sendVelocities()
            self.teleop.returnToOrigin()

    def setAlgorithm(self, algorithm ):
        self.algorithm=algorithm

    def getAlgorithm(self):
        return self.algorithm

    def setXYValues(self,newX,newY):
        myW=-newX*self.motors.getMaxW()
        myV=-newY*self.motors.getMaxV()
        self.motors.sendV(myV)
        self.motors.sendW(myW)
        #self.motors.sendVelocities(self.motors)

    def stopClicked(self):
        self.motors.sendV(0)
        self.motors.sendW(0)
        # self.motors.sendVelocities()
        self.teleop.returnToOrigin()
