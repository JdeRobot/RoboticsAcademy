import sys, math
import config
import comm
from math import pi as pi
import numpy as np
import cv2
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5 import QtGui, QtCore
from parallelIce.pose3dClient import Pose3DClient
import easyiceconfig as EasyIce
from gui.threadGUI import ThreadGUI


class MainWindowReferee(QWidget):

    updGUI=pyqtSignal()
    def __init__(self, pose3d, parent=None):
        super(MainWindowReferee, self).__init__(parent)
        
        layout = QGridLayout()
        self.time = timeWidget(self, pose3d)
        self.distance = distanceWidget(self, pose3d)
        self.speed = speedWidget(self, pose3d, self.time)
        self.angle = angleWidget(self, pose3d)
        self.mark = markWidget(self, pose3d, self.time, self.distance)

        layout.addWidget(self.time,0,0)
        layout.addWidget(self.distance,0,1)
        layout.addWidget(self.speed,1,0)
        layout.addWidget(self.angle,1,1)
        layout.addWidget(self.mark,2,1)
    
        vSpacer = QSpacerItem(30, 30, QSizePolicy.Ignored, QSizePolicy.Ignored)
        layout.addItem(vSpacer,1,0)
        
        self.setFixedSize(840,500);

        self.setLayout(layout)
        self.updGUI.connect(self.update)

    def update(self):
        self.distance.updateG()
        self.speed.updateG()
        self.angle.updateG()
        
        
class timeWidget(QWidget):

    time = pyqtSignal()
    def __init__(self,winParent, pose3d):    
        super(timeWidget, self).__init__()
        self.winParent=winParent
        self.seconds = 0
        self.secondsDrive = 0
        self.pose3d = pose3d
        
        hLayout = QHBoxLayout()
        
        timeLabel = QLabel("Time")
        self.lcd = QLCDNumber(self)
        self.lcd.setMaximumSize(100,50)
        hLayout.addWidget(timeLabel,0)
        hLayout.addWidget(self.lcd, 1)

        hSpacer = QSpacerItem(300, 30, QSizePolicy.Ignored, QSizePolicy.Ignored)
        hLayout.addItem(hSpacer)

        self.setLayout(hLayout)

        timer = QTimer(self)
        timer.start(1000)
        timer.timeout.connect(self.printTime)
        

        # get the palette
        palette = self.lcd.palette()

        # foreground color
        palette.setColor(palette.WindowText, QColor(85, 85, 255))
        # background color
        palette.setColor(palette.Background, QColor(0, 170, 255))
        # "light" border
        palette.setColor(palette.Light, QColor(255, 0, 0))
        # "dark" border
        palette.setColor(palette.Dark, QColor(0, 255, 0))

        # set the palette
        self.lcd.setPalette(palette)

    def printTime(self):
        if self.pose3d.getPose3d().x != 0 and self.pose3d.getPose3d().y != 0:
            self.secondsDrive += 1
        self.seconds += 1
        self.lcd.display(self.seconds)
              

class distanceWidget(QWidget):
    def __init__(self,winParent, pose3d):    
        super(distanceWidget, self).__init__()
        self.winParent=winParent
        self.pose3d = pose3d
        
        self.wWidth = 500
        self.wHeight = 500
        self.gWidth = 400
        self.gHeight = 400
        
        f = open("sensors/destiny.txt", "w")
        f.write("")
        f.close() 
        
        self.destinyString = "\n"
        self.destiny = 0
        self.distance = 0

        vLayout = QVBoxLayout()

        self.distanceLabel = QLabel("Distance between robot and target: " + 'Destination not yet selected')
        vLayout.addWidget(self.distanceLabel, 0)

        self.setLayout(vLayout)
        
        
    def RTx(self, angle, tx, ty, tz):
        RT = np.matrix([[1, 0, 0, tx], [0, math.cos(angle), -math.sin(angle), ty], [0, math.sin(angle), math.cos(angle), tz], [0,0,0,1]])
        return RT
        
    def RTy(self, angle, tx, ty, tz):
        RT = np.matrix([[math.cos(angle), 0, math.sin(angle), tx], [0, 1, 0, ty], [-math.sin(angle), 0, math.cos(angle), tz], [0,0,0,1]])
        return RT
    
    def RTz(self, angle, tx, ty, tz):
        RT = np.matrix([[math.cos(angle), -math.sin(angle), 0, tx], [math.sin(angle), math.cos(angle),0, ty], [0, 0, 1, tz], [0,0,0,1]])
        return RT
		
    def RTGridWorld(self):
        RTx = self.RTx(-pi, 0, 0, 0)
        RTz = self.RTz(pi/2, self.wWidth/2, self.wHeight/2, 0)
        return RTz*RTx
        
    def gridToWorld(self, gridX, gridY):
        # self.wWidth/self.gWidth is the scale
        gridX = float(gridX * self.wWidth)/float(self.gWidth)
        gridY = float(gridY * self.wHeight)/float(self.gHeight)
        orig_poses = np.matrix([[gridX], [gridY], [0], [-1]])
        final_poses = self.RTGridWorld() * orig_poses
        
        worldX = final_poses.flat[0]
        worldY = final_poses.flat[1]
        return (worldX, worldY)
        
    def calculateDistancePoints(self, destiny):
        x = self.pose3d.getPose3d().x
        y = self.pose3d.getPose3d().y
        dist = math.sqrt(pow((destiny[0]- x),2) + pow((destiny[1]- y),2))
        return dist
        
    def calculateDistance(self):
        f = open("sensors/destiny.txt")
        self.destinyString = f.readline()
        f.close()
        if self.destinyString != "" and self.distance == 0:
            destinyGrid = eval(self.destinyString)
            self.destiny = self.gridToWorld(destinyGrid[0], destinyGrid[1])
            self.distance = self.calculateDistancePoints(self.destiny)     


    def updateG(self):
        self.calculateDistance()
        if self.destinyString != "":
            self.distanceLabel.setText("Distance between robot (origin) and target: " + str(round(self.distance, 3)) + ' m')
        self.update()
        
        
class speedWidget(QWidget):

    def __init__(self,winParent, pose3d, time):    
        super(speedWidget, self).__init__()
        self.winParent=winParent
        self.pose3d = pose3d
        self.time = time
        self.secondsBefore = 0
        self.pose3dBeforeX = 0
        self.pose3dBeforeY = 0
        self.speed = 0
        
        self.rect = QRect(0, 0, 150,150)
        self.startAngle = 0 * 16
        self.spanAngle = 180 * 16     

        self.needle = QPolygon([
            QPoint(1, 0),
            QPoint(0, 1),
            QPoint(-1, 0),
            QPoint(0, 50)
            ])

        self.backColor = QColor('white')
        self.needleColor = QColor('blue')
        self.tickColor = QColor('red')
        
        vLayout = QVBoxLayout()
        self.speedLabel = QLabel("                  Speed: " + str(round(self.speed, 3)) + ' km/h')
        vLayout.addWidget(self.speedLabel, 1)
        self.setLayout(vLayout)
        
        
    def calculateSpeed(self):
        if self.secondsBefore != self.time.seconds:
            self.secondsBefore = self.time.seconds
            km_h = float(3600) / float(1000)
            xNow = self.pose3d.getPose3d().x
            yNow = self.pose3d.getPose3d().y
            speedX = float(abs(xNow - self.pose3dBeforeX) * km_h)
            speedY = float(abs(yNow - self.pose3dBeforeY) * km_h)
            self.speed = math.sqrt(pow(speedX,2) + pow(speedY,2))
            self.pose3dBeforeX = xNow
            self.pose3dBeforeY = yNow
            
            
    def paintEvent(self, event):        
        # Draw speedometer
        side = min(self.rect.width(), self.rect.height())
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.translate(self.rect.width()/2, self.rect.height()/2)
        painter.scale(side / 120.0, side / 120.0)
        
        # Background
        painter.setPen(QColor('black'))
        painter.setBrush(QBrush(self.backColor))
        painter.drawChord(self.rect, self.startAngle, self.spanAngle)
        painter.save()
        
        # Needle
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(self.needleColor))
        painter.translate(self.rect.width()/2, self.rect.height()/2)
        painter.rotate(90+(self.speed/10) * 15)
        painter.drawConvexPolygon(self.needle)
        painter.restore()
        
        # Tick marks
        painter.setPen(self.tickColor)
        painter.translate(self.rect.width()/2, self.rect.height()/2)
        painter.rotate(-15.0)
        for i in range(0, 11):
            painter.drawLine(75, 0, 72, 0)
            painter.rotate(-15.0)
            
        painter.end()
        
        self.speedLabel.setText("                   Speed: " + str(round(self.speed, 3)) + ' km/h')
        
        
    def updateG(self):
        self.calculateSpeed()
        self.update()
        
        
class angleWidget(QWidget):

    def __init__(self,winParent, pose3d):    
        super(angleWidget, self).__init__()
        self.winParent=winParent
        self.pose3d = pose3d
        self.angle = self.pose3d.getPose3d().yaw
        
        self.rect = QRect(0, 0, 150,150)
        self.startAngle = 0 * 16
        self.spanAngle = 180 * 16     

        self.needle = QPolygon([
            QPoint(1, 0),
            QPoint(0, 1),
            QPoint(-1, 0),
            QPoint(0, 50)
            ])

        self.backColor = QColor('white')
        self.needleColor = QColor('blue')
        self.tickColor = QColor('red')
        
        vLayout = QVBoxLayout()
        self.angleLabel = QLabel("                  Angle: " + str(round(self.angle, 3)) + ' radians')
        vLayout.addWidget(self.angleLabel, 1)
        self.setLayout(vLayout)
        
        
    def calculateAngle(self):
        self.angle = self.pose3d.getPose3d().yaw
            
            
    def paintEvent(self, event):        
        # Draw speedometer
        side = min(self.rect.width(), self.rect.height())
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.translate(self.rect.width()/2, self.rect.height()/2)
        painter.scale(side / 120.0, side / 120.0)
        
        # Background
        painter.setPen(QColor('black'))
        painter.setBrush(QBrush(self.backColor))
        painter.drawChord(self.rect, self.startAngle, self.spanAngle)
        painter.save()
        
        # Needle
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(self.needleColor))
        painter.translate(self.rect.width()/2, self.rect.height()/2)
        painter.rotate(180+(self.angle*1.9) * 15)
        painter.drawConvexPolygon(self.needle)
        painter.restore()
        
        # Tick marks
        painter.setPen(self.tickColor)
        painter.translate(self.rect.width()/2, self.rect.height()/2)
        painter.rotate(-15.0)
        for i in range(0, 11):
            painter.drawLine(75, 0, 72, 0)
            painter.rotate(-15.0)
            
        painter.end()
        
        self.angleLabel.setText("                   Angle: " + str(round(self.angle, 3)) + ' radians')
        
        
    def updateG(self):
        self.calculateAngle()
        self.update()


        
class markWidget(QWidget):
    def __init__(self,winParent,pose3d, time, distance):    
        super(markWidget, self).__init__()
        self.winParent=winParent
        self.pose3d = pose3d
        self.time = time
        self.dist = distance

        self.hLayout = QHBoxLayout()
        
        self.button = QPushButton('Show me my mark')
        self.button.clicked.connect(self.markFinal)
        self.hLayout.addWidget(self.button, 0)
        
        self.setLayout(self.hLayout)
        
    def markSpeed(self):
        MARK_MAX = 10
        SPEED_MAX = 2.6
        speed = float(self.dist.distance) / float(self.time.secondsDrive)
        mark = float(speed * MARK_MAX) / float(SPEED_MAX)
        if speed > SPEED_MAX:
            mark = 10
        return mark
        
    def markFinal(self):
        markSpeed = self.markSpeed()
        found = self.checkDestination()
        if found == True:
            xRobot = self.pose3d.getPose3d().x
            yRobot = self.pose3d.getPose3d().y
            xdest = self.dist.destiny[0]
            ydest = self.dist.destiny[1]
            
            if (abs(xRobot)<(abs(xdest)+2) and abs(xRobot)>(abs(xdest)-2)) and (abs(yRobot)<(abs(ydest)+2) and abs(yRobot)>(abs(ydest)-2)):
                markArrive = 8
            elif (abs(xRobot)<(abs(xdest)+4) and abs(xRobot)>(abs(xdest)-4)) and (abs(yRobot)<(abs(ydest)+4) and abs(yRobot)>(abs(ydest)-4)):
                markArrive = 7.5
            else:
                markArrive = 7
            mark = markArrive + markSpeed * 0.2
            markLabel = QLabel('Final Mark: ' + str(mark))
        else:
            markLabel = QLabel('The destination has not been reached')
        self.hLayout.addWidget(markLabel, 0)
        f = open("sensors/destiny.txt", "w")
        f.write("")
        f.close()              
    
    def checkDestination(self):
        found = False
        xRobot = self.pose3d.getPose3d().x
        yRobot = self.pose3d.getPose3d().y
        xdest = self.dist.destiny[0]
        ydest = self.dist.destiny[1]
        if (abs(xRobot)<(abs(xdest)+5) and abs(xRobot)>(abs(xdest)-5)) and (abs(yRobot)<(abs(ydest)+5) and abs(yRobot)>(abs(ydest)-5)):
            found = True
        return found
        

    def updateG(self):
        self.update()
        
        
        
if __name__ == "__main__":
    
    app = QApplication(sys.argv)
    cfg = config.load(sys.argv[1])
    #starting comm
    jdrc= comm.init(cfg, 'Referee')

    pose3d = jdrc.getPose3dClient("Referee.Pose3D")

    myGUI = MainWindowReferee(pose3d)
    myGUI.show()
    t2 = ThreadGUI(myGUI)
    t2.daemon=True
    t2.start()
    sys.exit(app.exec_())
