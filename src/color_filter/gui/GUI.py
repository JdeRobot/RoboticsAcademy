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


from PyQt4 import QtGui,QtCore
from gui.ui_gui import Ui_MainWindow
from gui.teleopWidget import TeleopWidget
from gui.cameraWidget import CameraWidget
from gui.communicator import Communicator
from gui.sensorsWidget import SensorsWidget
from gui.colorFilterWidget import  ColorFilterWidget

class MainWindow(QtGui.QMainWindow, Ui_MainWindow):
    
    updGUI=QtCore.pyqtSignal()
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.teleop=TeleopWidget(self)
        self.tlLayout.addWidget(self.teleop)
        self.teleop.setVisible(True)

        self.record = False

        self.updGUI.connect(self.updateGUI)
        
        self.cameraCheck.stateChanged.connect(self.showCameraWidget)
        self.sensorsCheck.stateChanged.connect(self.showSensorsWidget)
        self.colorFilterCheck.stateChanged.connect(self.showColorFilterWidget)
        
        self.rotationDial.valueChanged.connect(self.rotationChange)
        self.altdSlider.valueChanged.connect(self.altitudeChange)
        
        self.cameraWidget=CameraWidget(self)
        self.sensorsWidget=SensorsWidget(self)
        self.colorFilterWidget=ColorFilterWidget(self)

        self.cameraCommunicator=Communicator()
        self.colorFilterCommunicator=Communicator()
        self.trackingCommunicator = Communicator()

        self.stopButton.clicked.connect(self.stopClicked)
        self.playButton.clicked.connect(self.playClicked)
        self.resetButton.clicked.connect(self.resetClicked)
        self.takeoffButton.clicked.connect(self.takeOffClicked)
        self.takeoff=False
        self.reset=False
      
    def setSensor(self,sensor):
        self.sensor=sensor
           
    def getSensor(self):
        return self.sensor
    
    def updateGUI(self):
        self.cameraWidget.imageUpdate.emit()
        self.sensorsWidget.sensorsUpdate.emit()
        self.colorFilterWidget.imageUpdate.emit()
    
    def playClicked(self):
        if self.record == True:
            self.sensor.record(True)
        self.sensor.setPlayButton(True)
    
    def stopClicked(self):
        if self.record == True:
            self.sensor.record(False)
        self.sensor.setPlayButton(False)
        self.rotationDial.setValue(self.altdSlider.maximum()/2)
        self.altdSlider.setValue(self.altdSlider.maximum()/2)
        self.sensor.sendCMDVel(0,0,0,0,0,0)
        self.teleop.stopSIG.emit()
    
    def takeOffClicked(self):
        if(self.takeoff==True):
            self.takeoffButton.setText("Take Off")
            self.sensor.land()
            self.takeoff=False
        else:
            self.takeoffButton.setText("Land")    
            self.sensor.takeoff()
            self.takeoff=True

    def resetClicked(self):
        if self.reset == True:
            self.resetButton.setText("Reset")
            self.sensor.reset()
            self.reset=False
        else:
            self.resetButton.setText("Unreset")
            self.sensor.reset()
            self.reset=True
        
    def showCameraWidget(self,state):
        if state == QtCore.Qt.Checked:
            self.cameraWidget.show()
        else:
            self.cameraWidget.close()
            
    def closeCameraWidget(self):
        self.cameraCheck.setChecked(False)

    def showColorFilterWidget(self,state):
        if state == QtCore.Qt.Checked:
            self.colorFilterWidget.show()
        else:
            self.colorFilterWidget.close()

    def closeColorFilterWidget(self):
        self.colorFilterCheck.setChecked(False)

    def showSensorsWidget(self,state):
        if state == QtCore.Qt.Checked:
            self.sensorsWidget.show()           
        else:
            self.sensorsWidget.close() 

    def closeSensorsWidget(self):
        self.sensorsCheck.setChecked(False)
    
    def rotationChange(self,value):
        value=(1.0/(self.rotationDial.maximum()/2))*(value - (self.rotationDial.maximum()/2))
        self.rotValue.setText(unicode(value))  
        self.sensor.setYaw(value)
        self.sensor.sendVelocities()

    def altitudeChange(self,value):
        value=(1.0/(self.altdSlider.maximum()/2))*(value - (self.altdSlider.maximum()/2))
        self.altdValue.setText(unicode(value))
        self.sensor.setVZ(value)
        self.sensor.sendVelocities()

    def setXYValues(self,newX,newY):
        self.XValue.setText(unicode(newX))
        self.YValue.setText(unicode(newY))
        self.sensor.setVX(-newY)
        self.sensor.setVY(-newX)
        self.sensor.sendVelocities()

