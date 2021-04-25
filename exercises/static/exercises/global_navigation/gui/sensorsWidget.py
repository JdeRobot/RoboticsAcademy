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
from PyQt4 import QtGui, QtCore
from gui.speedoMeter import SpeedoMeter
from gui.attitudeIndicator import AttitudeIndicator 
from PyQt4 import Qt
import PyQt4.Qwt5 as Qwt
import math


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

class SensorsWidget(QtGui.QWidget):
    
    sensorsUpdate=QtCore.pyqtSignal()
    
    def __init__(self,winParent):      
        super(SensorsWidget, self).__init__()
        self.winParent=winParent
        self.sensorsUpdate.connect(self.updateSensors)
        self.initUI()
        
    def initUI(self):
        
        self.setMinimumSize(660,450)
        self.setMaximumSize(660,450)
        
        self.setWindowTitle("Sensors")
        
        self.horizon=self.__createDial(0)
        self.horizon.resize(200,200)
        self.horizon.move(60,20) 

       	self.pitchLabel=QtGui.QLabel('Pitch:',self)
        self.pitchLabel.move(300,140)
        self.pitchValueLabel=QtGui.QLabel('-180',self)
        self.pitchValueLabel.resize(60,21)
        self.pitchValueLabel.move(340,140)
        
        self.rollLabel=QtGui.QLabel('Roll:',self)
        self.rollLabel.move(300,160)
        self.rollValueLabel=QtGui.QLabel('180',self)
        self.rollValueLabel.resize(60,21)
        self.rollValueLabel.move(340,160)

        self.compass=self.__createCompass(1)
        self.compass.resize(120,120)
        self.compass.move(280,10)
        self.compass.show()

        self.yawLabel=QtGui.QLabel('Yaw:',self)
        self.yawLabel.move(300,180)
        self.yawValueLabel=QtGui.QLabel('45',self)
        self.yawValueLabel.resize(60,21)
        self.yawValueLabel.move(340,180)

        self.altd=self.__createDial(2)
        self.altd.resize(150,150)
        self.altd.move(420,50)


        self.battery=Qwt.QwtThermo(self)
        self.battery.setMaxValue(100.0)
        self.battery.setMinValue(0.0)
        self.battery.setPipeWidth(10)
        
        self.battery.move(580,10)
        self.battery.resize(56,241)
        self.batteryLabel=QtGui.QLabel('Battery (%)',self)
        self.batteryLabel.move(580,251)

        self.velLinX=self.__createDial(1)
        self.velLinX.resize(150,150)
        self.velLinX.move(60,270)
        self.velXLabel = QtGui.QLabel('Linear X (m/s)',self)        
        self.velXLabel.move(95,420)

        
        self.velLinY=self.__createDial(1)
        self.velLinY.resize(150,150)
        self.velLinY.move(240,270)
        self.velYLabel = QtGui.QLabel('Linear Y (m/s)',self)
        self.velYLabel.move(275,420)             
        
        self.velLinZ=self.__createDial(1)
        self.velLinZ.resize(150,150)
        self.velLinZ.setLabel("8 m/s")
        self.velLinZ.move(420,270)
        self.velZLabel = QtGui.QLabel('Linear Z (m/s)',self)
        self.velZLabel.move(455,420)


        self.__speed_offset = 0.8
        self.__angle_offset = 0.05
        self.__gradient_offset = 0.005
        
        
    def updateSensors(self):
        pose=self.winParent.getSensor().getPose3D()

        if pose != None:
            qw=pose.q0
            qx=pose.q1
            qy=pose.q2
            qz=pose.q3
            self.drawAltd(pose.z)
            self.drawYawValues(self.quatToYaw(qw,qx,qy,qz)*180/math.pi)
            self.drawPitchRollValues(self.quatToPitch(qw,qx,qy,qz)*180/math.pi,self.quatToRoll(qw,qx,qy,qz)*180/math.pi)


        navdata=self.winParent.getSensor().getNavdata()

        if navdata != None:
            self.battery.setValue(navdata.batteryPercent)
            self.drawVelocities(navdata.vx,navdata.vy,navdata.vz)
    
  
    def drawYawValues(self,degress):
        value="{0:.2f}".format(degress)
        self.yawValueLabel.setText(unicode(value))
        self.compass.setValue(degress)

    def drawAltd(self, meters):

        if(meters>=10 and meters<100):
            self.altd.setValue(meters%10)
        elif (meters>=100 and meters<1000):
            self.altd.setValue(meters%100)          
        else:
            self.altd.setValue(meters)

        altLabel="{0:.0f}".format(meters)+' m'
        self.altd.setLabel(altLabel)

    def drawPitchRollValues(self,pitch,roll):
        if(pitch>0 and pitch<=90):
            result=pitch/90
            result=-result
        elif (pitch<0 and pitch>=-90):
            result=pitch/-90
        else:
            result=0.0
    
        self.horizon.setGradient(result)
        self.horizon.setAngle(-roll)
        pitchValue="{0:.2f}".format(pitch)
        rollValue="{0:.2f}".format(roll)
        self.pitchValueLabel.setText(unicode(pitchValue))
        self.rollValueLabel.setText(unicode(rollValue))


    def drawVelocities(self,vx,vy,vz):

        vx=math.fabs(vx)
        vx/=1000.0
        self.velLinX.setValue(vx)
        vx=math.fabs(vx)
        vxLabel="{0:.0f}".format(vx)+' m/s'
        self.velLinX.setLabel(vxLabel)
        
        vy=math.fabs(vy)
        vy/=1000.0
        self.velLinY.setValue(vy)
        vyLabel="{0:.0f}".format(vy)+' m/s'
        self.velLinY.setLabel(vyLabel)

        vz=math.fabs(vz)
        vz/=1000.0
        self.velLinZ.setValue(vz)
        vzLabel="{0:.0f}".format(vz)+' m/s'
        self.velLinZ.setLabel(vzLabel)
        
   
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
        
    def __createDial(self, pos):
        dial = None
        if pos == 0:
            self.__ai = AttitudeIndicator(self)
            dial = self.__ai
        elif pos == 1:
            self.__speedo = SpeedoMeter(self)
            self.__speedo.setRange(0.0, 8.0)
            self.__speedo.setLabel("0")
            self.__speedo.setOrigin(180)
            self.__speedo.setScaleArc(0.0,270.0)
            self.__speedo.setScale(-1, 2, 1.0)
            dial = self.__speedo   
        elif pos == 2:
            self.__speedo = SpeedoMeter(self)
            self.__speedo.setRange(0.0, 10.0)
            self.__speedo.setLabel("m")
            self.__speedo.setOrigin(-90)
            self.__speedo.setScaleArc(0.0,360.0)
            self.__speedo.setScale(-1, 2, 1)
            dial = self.__speedo                        

        if dial:
            dial.setReadOnly(True)
            dial.scaleDraw().setPenWidth(3)
            dial.setLineWidth(4)
            dial.setFrameShadow(Qwt.QwtDial.Sunken)

        return dial    
    
    def __createCompass(self, pos):

        palette = Qt.QPalette()
        for colorRole in colorRoleList:
            palette.setColor(colorRole, Qt.QColor())

        palette.setColor(
            Qt.QPalette.Base,
            self.palette().color(self.backgroundRole()).light(120))
        palette.setColor(
            Qt.QPalette.Foreground,
            palette.color(Qt.QPalette.Base))

        compass = Qwt.QwtCompass(self)
        compass.setLineWidth(4)
        if pos < 3:
            compass.setFrameShadow(Qwt.QwtCompass.Sunken)
        else:
            compass.setFrameShadow(Qwt.QwtCompass.Raised)

        if pos == 0:
            compass.setLabelMap({0.0: "N",
                                 90.0: "E",
                                 180.0: "S",
                                 270.0: "W"})
            
            rose = Qwt.QwtSimpleCompassRose(4, 1)
            compass.setRose(rose)
            compass.setNeedle(
                Qwt.QwtCompassWindArrow(Qwt.QwtCompassWindArrow.Style2))
            compass.setValue(60.0)
        elif pos == 1:
            compass.setLabelMap({0.0: "",
                                 90.0: "",
                                 180.0: "",
                                 270.0: ""})
            
            compass.setScaleOptions(Qwt.QwtDial.ScaleBackbone
                        | Qwt.QwtDial.ScaleTicks
                        | Qwt.QwtDial.ScaleLabel)
            compass.setScaleTicks(0, 0, 3)
            
            compass.setNeedle(Qwt.QwtCompassMagnetNeedle(
                Qwt.QwtCompassMagnetNeedle.TriangleStyle,
                Qt.Qt.white,
                Qt.Qt.red))
            compass.setValue(220.0)            
 
        newPalette = compass.palette()
        for colorRole in colorRoleList:
            if palette.color(colorRole).isValid():
                for colorGroup in colorGroupList:
                    newPalette.setColor(
                        colorGroup, colorRole, palette.color(colorRole))

        for colorGroup in colorGroupList:
            light = newPalette.color(
                colorGroup, Qt.QPalette.Base).light(170)
            dark = newPalette.color(
                colorGroup, Qt.QPalette.Base).dark(170)
            if compass.frameShadow() == Qwt.QwtDial.Raised:
                mid = newPalette.color(
                    colorGroup, Qt.QPalette.Base).dark(110)
            else:
                mid = newPalette.color(
                    colorGroup, Qt.QPalette.Base).light(110)

            newPalette.setColor(colorGroup, Qt.QPalette.Dark, dark)
            newPalette.setColor(colorGroup, Qt.QPalette.Mid, mid)
            newPalette.setColor(colorGroup, Qt.QPalette.Light, light)

        compass.setPalette(newPalette)
        compass.setReadOnly(True)
        return compass

        

