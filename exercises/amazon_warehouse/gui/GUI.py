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
#       Alberto Martin Florido <almartinflorido@gmail.com>
#       Samuel Rey Escudero <samuel.rey.escudero@gmail.com>



from PyQt5 import QtGui,QtCore
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QMainWindow
from gui.ui_gui import Ui_MainWindow
from gui.teleopWidget import TeleopWidget
from gui.communicator import Communicator
from gui.mapWidget import Map
from gui.logoWidget import LogoWidget
import rospy
from std_msgs.msg import Float32

class MainWindow(QMainWindow, Ui_MainWindow):
    
    getPathSig = QtCore.pyqtSignal(list)
    updGUI = QtCore.pyqtSignal()
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.teleop=TeleopWidget(self)
        self.tlLayout.addWidget(self.teleop)
        self.map = Map(self)
        self.mapLayout.addWidget(self.map)
        self.teleop.setVisible(True)

        self.logo = LogoWidget(self, self.logoLayout.parent().width(), self.logoLayout.parent().height())
        self.logoLayout.addWidget(self.logo)
        self.logo.setVisible(True)

        self.updGUI.connect(self.updateGUI)
        self.getPathButton.clicked.connect(self.getPathClicked)
        self.playButton.clicked.connect(self.playClicked)
        self.playButton.setCheckable(True)
        #self.stopButton.clicked.connect(self.stopClicked)

        # plannerList = ['RRTstar', 'SORRTstar','BITstar', 'FMTstar', 'InformedRRTstar', 'PRMstar']
        # self.plannerBox.addItems(plannerList)
        # objectiveList = ['PathClearance', 'PathLength', 'ThresholdPathLength', 'WeightedLengthAndClearanceCombo']
        # self.objectiveBox.addItems(objectiveList)
        self.liftDropButton.clicked.connect(self.liftDropExecute)
        self.liftDropButton.setCheckable(True)
        self.jointForce = 0
        self.pub = rospy.Publisher('amazon_warehouse_robot/joint_cmd', Float32, queue_size=10)

        self.gotoPointButton.clicked.connect(self.gotoPointExecute)
        self.gotoPointButton.setCheckable(True)

    def setSensor(self, sensor):
        self.sensor = sensor
           
    def getSensor(self):
        return self.sensor

    def getMotors(self):
        return self.motors

    def setMotors(self,motors):
        self.motors=motors

    def setAlgorithm(self, algorithm ):
        self.algorithm=algorithm

    def setVelocity(self, vel):
        self.vel = vel

    def getVelocity(self):
        return self.vel

    def getAlgorithm(self):
        return self.algorithm
    
    def setGrid(self, grid):
        self.grid = grid
        self.grid.setMap(self.map.map)

    def updateGUI(self):
        self.map.updateMap(self.grid)
    
    def width(self):
        return self.map.width

    def height(self):
        return self.map.height

    def worldWidth(self):
        return self.map.worldWidth

    def worldHeight(self):
        return self.map.worldHeight

    def origX(self):
        return self.map.originX

    def origY(self):
        return self.map.originY

    def mapAngle(self):
        return self.map.mapAngle

    def getPathClicked(self):
        pList = []
        # pList.append(self.getPlanner())
        #pList.append(self.getObjective())
        # pList.append(self.getRunTime())
        self.getPathSig.emit(pList)

    def playClicked(self):
        if self.playButton.isChecked():
            icon = QtGui.QIcon()
            self.playButton.setText("Stop Code")
            self.playButton.setStyleSheet("background-color: #ec7063")
            icon.addPixmap(QtGui.QPixmap(":/images/stop.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.playButton.setIcon(icon)
            self.algorithm.play()
        else:
            icon = QtGui.QIcon()
            self.playButton.setStyleSheet("background-color: #7dcea0")
            icon.addPixmap(QtGui.QPixmap(":/images/play.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.playButton.setIcon(icon)
            self.playButton.setText("Play Code")
            self.algorithm.stop()
            self.setXYValues(0, 0)
            self.teleop.stopSIG.emit()

    def setXYValues(self,newW,newV):
        # self.WValue.setText(str(newW))
        # self.VValue.setText(str(-newV))
        myW = newW * self.vel.getMaxW()
        myV = -newV * self.vel.getMaxV()
        self.vel.setV(myV)
        self.vel.setW(myW)

    def getPlanner(self):
        planner = self.plannerBox.currentText()
        return planner

    def getObjective(self):
        objective = self.objectiveBox.currentText()
        return objective
    
    def getRunTime(self):
        runtime = str(self.runtimeBox.value())
        return runtime

    def liftDropExecute(self):
        #print ('Lift/Drop Button Clicked')
        if self.jointForce != 25:
            self.jointForce = 25
            self.pub.publish(self.jointForce)
            self.liftDropButton.setText("Drop")
            print ('Platform Lifted!')
        else:
            self.jointForce = 0
            self.pub.publish(self.jointForce)
            self.liftDropButton.setText("Lift")
            print ('Platform Dropped!')

    def gotoPointExecute(self):
        self.algorithm.setNewPalletFlag(self.playButton.isChecked())

    def setDestinyXYValues(self, newX, newY):
        self.XValue.setText(str(newX))
        self.YValue.setText(str(newY))

    def setPositionXYValues(self, newX, newY):
        self.VValue.setText(str(newX))
        self.WValue.setText(str(newY))
