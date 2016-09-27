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



from PyQt4 import QtGui,QtCore
from gui.ui_gui import Ui_MainWindow
from gui.teleopWidget import TeleopWidget
from gui.communicator import Communicator
from gui.mapWidget import Map
from colorFilterWidget import ColorFilterWidget

class MainWindow(QtGui.QMainWindow, Ui_MainWindow):
    
    getPathSig = QtCore.pyqtSignal()
    updGUI=QtCore.pyqtSignal()
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.teleop=TeleopWidget(self)
        self.colorFilterWidget=ColorFilterWidget(self)
        self.tlLayout.addWidget(self.teleop)
        self.map = Map(self)
        self.mapLayout.addWidget(self.map)
        self.teleop.setVisible(True)

        self.updGUI.connect(self.updateGUI)
        self.getPathButton.clicked.connect(self.getPathClicked)
        self.playButton.clicked.connect(self.playClicked)
        self.stopButton.clicked.connect(self.stopClicked)
        self.colorFilter.stateChanged.connect(self.showColorFilterWidget)
      
    def setSensor(self,sensor):
        self.sensor=sensor
           
    def getSensor(self):
        return self.sensor

    def getMotors(self):
        return self.motors

    def setMotors(self,motors):
        self.motors=motors

    def setAlgorithm(self, algorithm ):
        self.algorithm=algorithm

    def getAlgorithm(self):
        return self.algorithm
    
    def setGrid(self, grid):
        self.grid = grid
        self.grid.setMap(self.map.map)

    def updateGUI(self):
        self.map.updateMap(self.grid)
        self.colorFilterWidget.imageUpdate.emit()
    
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
        self.getPathSig.emit()

    def playClicked(self):
        print "play clicked"
        self.algorithm.play()
    
    def stopClicked(self):
        print "Stop clicked"
        self.algorithm.stop()
        self.setXYValues(0, 0)
        self.teleop.stopSIG.emit()

    def setXYValues(self,newW,newV):
        self.WValue.setText(unicode(newW))
        self.VValue.setText(unicode(-newV))
        myW=newW*self.motors.getMaxW()
        myV=-newV*self.motors.getMaxV()
        self.motors.setV(myV)
        self.motors.setW(myW)
        self.motors.sendVelocities()

    def closeColorFilterWidget(self):
        self.colorFilter.setChecked(False)

    def showColorFilterWidget(self, state):
        if state == QtCore.Qt.Checked:
            self.colorFilterWidget.show()
        else:
            self.colorFilterWidget.close()