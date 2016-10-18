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
from PyQt5.QtWidgets import QWidget, QLabel
from PyQt5.QtCore import pyqtSignal
#from gui.speedoMeter import SpeedoMeter
#from gui.attitudeIndicator import AttitudeIndicator 
from PyQt5 import Qt
from qfi import qfi_ADI, qfi_ALT, qfi_SI, qfi_HSI
import math


class SensorsWidget(QWidget):
    
    sensorsUpdate=pyqtSignal()
    
    def __init__(self,winParent):      
        super(SensorsWidget, self).__init__()
        self.winParent=winParent
        self.sensorsUpdate.connect(self.updateSensors)
        self.initUI()
        
    def initUI(self):
        
        self.setMinimumSize(660,450)
        self.setMaximumSize(660,450)
        
        self.setWindowTitle("Sensors")
        
        self.horizon=qfi_ADI.qfi_ADI(self)
        self.horizon.resize(200,200)
        self.horizon.move(60,20) 
        #self.addWidget(self.horizon)

       	self.pitchLabel=QLabel('Pitch:',self)
        self.pitchLabel.move(300,140)
        self.pitchValueLabel=QLabel('-180',self)
        self.pitchValueLabel.resize(60,21)
        self.pitchValueLabel.move(340,140)
        
        self.rollLabel=QLabel('Roll:',self)
        self.rollLabel.move(300,160)
        self.rollValueLabel=QLabel('180',self)
        self.rollValueLabel.resize(60,21)
        self.rollValueLabel.move(340,160)

        self.compass=qfi_HSI.qfi_HSI(self)
        self.compass.resize(120,120)
        self.compass.move(280,10)
        self.compass.show()

        self.yawLabel=QLabel('Yaw:',self)
        self.yawLabel.move(300,180)
        self.yawValueLabel=QLabel('45',self)
        self.yawValueLabel.resize(60,21)
        self.yawValueLabel.move(340,180)

        self.altd=qfi_ALT.qfi_ALT(self)
        self.altd.resize(150,150)
        self.altd.move(420,50)


        #self.battery=Qwt.QwtThermo(self)
        #self.battery.setMaxValue(100.0)
        #self.battery.setMinValue(0.0)
        #self.battery.setPipeWidth(10)
        
        #self.battery.move(580,10)
        #self.battery.resize(56,241)
        self.batteryLabel=QLabel('Battery (%)',self)
        self.batteryLabel.move(580,251)

        self.velLinX=qfi_SI.qfi_SI(self)
        self.velLinX.resize(150,150)
        self.velLinX.move(60,270)
        self.velXLabel = QLabel('Linear X (m/s)',self)        
        self.velXLabel.move(95,420)

        
        self.velLinY=qfi_SI.qfi_SI(self)
        self.velLinY.resize(150,150)
        self.velLinY.move(240,270)
        self.velYLabel = QLabel('Linear Y (m/s)',self)
        self.velYLabel.move(275,420)             
        
        self.velLinZ=qfi_SI.qfi_SI(self)
        self.velLinZ.resize(150,150)
        #self.velLinZ.setLabel("8 m/s")
        self.velLinZ.move(420,270)
        self.velZLabel = QLabel('Linear Z (m/s)',self)
        self.velZLabel.move(455,420)


        self.__speed_offset = 0.8
        self.__angle_offset = 0.05
        self.__gradient_offset = 0.005
        
        
    def updateSensors(self):
        pose=self.winParent.getPose3D().getPose3D()

        if pose != None:
            qw=pose.q0
            qx=pose.q1
            qy=pose.q2
            qz=pose.q3
            self.drawAltd(pose.z)
            self.drawYawValues(self.quatToYaw(qw,qx,qy,qz)*180/math.pi)
            self.drawPitchRollValues(self.quatToPitch(qw,qx,qy,qz)*180/math.pi,self.quatToRoll(qw,qx,qy,qz)*180/math.pi)


        navdata=self.winParent.getNavData().getNavdata()

        if navdata != None:
            #self.battery.setValue(navdata.batteryPercent)
            self.drawVelocities(navdata.vx,navdata.vy,navdata.vz)
    
  
    def drawYawValues(self,degress):
        value="{0:.2f}".format(degress)
        self.yawValueLabel.setText(unicode(value))
        self.compass.setHeading(degress)
        self.compass.viewUpdate.emit()

    def drawAltd(self, meters):

        self.altd.setValue(meters)
        self.altd.viewUpdate.emit()

        #altLabel="{0:.0f}".format(meters)+' m'
        #self.altd.setLabel(altLabel)

    def drawPitchRollValues(self,pitch,roll):
        if(pitch>0 and pitch<=90):
            result=pitch/90
            result=-result
        elif (pitch<0 and pitch>=-90):
            result=pitch/-90
        else:
            result=0.0
    
        self.horizon.setPitch(pitch)
        self.horizon.setRoll(-roll)
        self.horizon.viewUpdate.emit()
        pitchValue="{0:.2f}".format(pitch)
        rollValue="{0:.2f}".format(roll)
        self.pitchValueLabel.setText(unicode(pitchValue))
        self.rollValueLabel.setText(unicode(rollValue))


    def drawVelocities(self,vx,vy,vz):

        vx=math.fabs(vx)
        vx/=1000.0
        self.velLinX.setSpeed(vx)
        self.velLinX.viewUpdate.emit()
        vx=math.fabs(vx)
        vxLabel="{0:.0f}".format(vx)+' m/s'
        #self.velLinX.setLabel(vxLabel)
        
        vy=math.fabs(vy)
        vy/=1000.0
        self.velLinY.setSpeed(vy)
        self.velLinY.viewUpdate.emit()
        vyLabel="{0:.0f}".format(vy)+' m/s'
        #self.velLinY.setLabel(vyLabel)

        vz=math.fabs(vz)
        vz/=1000.0
        self.velLinZ.setSpeed(vz)
        self.velLinZ.viewUpdate.emit()
        vzLabel="{0:.0f}".format(vz)+' m/s'
        #self.velLinZ.setLabel(vzLabel)
        
   
    def quatToRoll(self,qw,qx,qy,qz):
        rotateXa0=2.0*(qy*qz + qw*qx)
        rotateXa1=qw*qw - qx*qx - qy*qy + qz*qz
        rotateX=0.0
        
        if(rotateXa0 != 0.0 and rotateXa1 !=0.0):
            rotateX=math.atan2(rotateXa0, rotateXa1)
            
        return rotateX

    def quatToPitch(self,qw,qx,qy,qz):
        rotateYa0=-2.0*(qx*qz - qw*qy)
        rotateY=0.0
        if(rotateYa0>=1.0):
            rotateY=math.pi/2.0
        elif(rotateYa0<=-1.0):
            rotateY=-math.pi/2.0
        else:
            rotateY=math.asin(rotateYa0)
        
        return rotateY


    def quatToYaw(self,qw,qx,qy,qz):
        rotateZa0=2.0*(qx*qy + qw*qz)
        rotateZa1=qw*qw + qx*qx - qy*qy - qz*qz
        rotateZ=0.0
        if(rotateZa0 != 0.0 and rotateZa1 != 0.0):
            rotateZ=math.atan2(rotateZa0,rotateZa1)
        
        return rotateZ                
        
    def closeEvent(self, event):
        self.winParent.closeSensorsWidget()

        

