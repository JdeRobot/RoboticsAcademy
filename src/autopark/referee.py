import sys, math
from math import pi as pi
import numpy as np
import cv2
from PyQt5.QtCore import QPoint, QRect, QSize, Qt, QPointF, QRectF, pyqtSignal, QTimer
from PyQt5.QtGui import (QBrush, QConicalGradient, QLinearGradient, QPainter, QPainterPath, QPalette, QPen, QPixmap, QPolygon, QRadialGradient, QColor, QTransform, QPolygonF, QKeySequence, QIcon)
from PyQt5.QtWidgets import (QApplication, QProgressBar, QCheckBox, QComboBox, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QSpinBox, QWidget, QPushButton, QSpacerItem, QSizePolicy, QLCDNumber )
from PyQt5 import QtGui, QtCore
from parallelIce.pose3dClient import Pose3DClient
from parallelIce.laserClient import LaserClient
import easyiceconfig as EasyIce
from gui.threadGUI import ThreadGUI

class MainWindow(QWidget):

    updGUI=pyqtSignal()
    def __init__(self, pose3d, laser1, laser2, laser3, parent=None):
        super(MainWindow, self).__init__(parent)
        
        layout = QGridLayout()
        self.cheese = cheeseWidget(self, pose3d)
        self.time = timeWidget(self)
        self.quality = qualityWidget(self, laser1, laser2, laser3)
        self.distance = distanceWidget(self, pose3d)
        self.mark = markWidget(self,pose3d, self.time, self.quality, self.distance)
        self.logo = logoWidget(self)
        layout.addWidget(self.cheese,1,0)
        layout.addWidget(self.time,0,0)
        layout.addWidget(self.distance,0,2)
        layout.addWidget(self.quality,1,2)
        layout.addWidget(self.mark,0,1)
        layout.addWidget(self.logo,2,2)
    
        vSpacer = QSpacerItem(30, 50, QSizePolicy.Ignored, QSizePolicy.Ignored)
        layout.addItem(vSpacer,1,0)
        
        self.setFixedSize(940,640);

        self.setLayout(layout)
        self.updGUI.connect(self.update)

    def update(self):
        self.cheese.updateG()
        self.distance.updateG()
        self.quality.updateG()
        self.mark.updateG()
        

class logoWidget(QWidget):
    def __init__(self, winParent):
        super(logoWidget, self).__init__()
        self.winParent=winParent
        self.logo = cv2.imread("resources/logo_jderobot1.png",cv2.IMREAD_UNCHANGED)
        self.logo = cv2.resize(self.logo, (100, 100))
        image = QtGui.QImage(self.logo.data, self.logo.shape[1], self.logo.shape[0],  QtGui.QImage.Format_ARGB32);
        self.pixmap = QtGui.QPixmap.fromImage(image)
        self.height = self.pixmap.height()
        self.width = self.pixmap.width()
        self.mapWidget = QLabel(self)
        self.mapWidget.setPixmap(self.pixmap)
        self.mapWidget.resize(self.width, self.height)
        self.setMinimumSize(100,100)
        
        
class qualityWidget(QWidget):
    def __init__(self,winParent, laser1, laser2, laser3):    
        super(qualityWidget, self).__init__()
        self.winParent=winParent
        self.laser1 = laser1
        self.laser2 = laser2
        self.laser3 = laser3
        self.numCrash = 0
        self.MAX_CRASH = 1000

        vLayout = QVBoxLayout()
        crashLabel = QLabel("Crash:")
        self.bar = QProgressBar()
        self.bar.setValue(self.numCrash)
        st = "QProgressBar::chunk {background-color: #ff0000;}\n QProgressBar {border: 1px solid grey;border-radius: 2px;text-align: center;background: #eeeeee;}"
        self.bar.setStyleSheet(st)
        self.bar.setTextVisible(False)
        vLayout.addWidget(crashLabel, 0)
        vLayout.addWidget(self.bar, 0)

        vSpacer = QSpacerItem(30, 80, QSizePolicy.Ignored, QSizePolicy.Ignored)
        vLayout.addItem(vSpacer)

        self.setLayout(vLayout)
        
        
    def get_laser_distance(self, laser):
        DIST = 15
        maxAngle = 180
        crash = False
        for i in range(0, maxAngle+1):
            # Distance in millimeters, we change to cm
            laserI = float(laser.distanceData[i])/float(10)
            if i != 0 and i != 180:
                if laserI <= DIST:
                    crash = True
        return crash
                    

    def updateG(self):
        laser_data_Front = self.laser1.getLaserData()
        laser_data_Rear = self.laser2.getLaserData()
        laser_data_Right = self.laser3.getLaserData()
        crashFront = self.get_laser_distance(laser_data_Front)
        crashRear = self.get_laser_distance(laser_data_Rear)
        crashRight = self.get_laser_distance(laser_data_Right)
        if crashFront or crashRear or crashRight:
            self.numCrash = self.numCrash + 1
        percentajeCrash = self.numCrash * 100/self.MAX_CRASH
        self.bar.setValue(self.numCrash)
        self.update()


class distanceWidget(QWidget):
    def __init__(self,winParent, pose3d):    
        super(distanceWidget, self).__init__()
        self.winParent=winParent
        self.pose3d = pose3d
        self.distFrontFinal = 0
        self.distRearFinal = 0
        self.distanceSidewalk = 0

        vLayout = QVBoxLayout()

        self.distances()

        distancesLabel = QLabel("Distances:")
        self.distanceFrontalLabel = QLabel("Front distance: " + str(round(self.distFrontFinal, 3)) + ' m')
        self.distanceRearLabel = QLabel("Rear distance: " + str(round(self.distRearFinal, 3)) + ' m')
        self.distanceSidewalkLabel = QLabel("Distance to the sidewalk: " + str(round(self.distanceSidewalk, 3)) + ' m')
        vLayout.addWidget(distancesLabel, 0)
        vLayout.addWidget(self.distanceFrontalLabel, 0)
        vLayout.addWidget(self.distanceRearLabel, 0)
        vLayout.addWidget(self.distanceSidewalkLabel, 0)

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

    def RTCar(self):
        yaw = self.pose3d.getYaw()
        RTz = self.RTz(yaw, 0, 0, 0)
        return RTz


    def distancePoint2Segment(self, A, B, C):
        # Segment: A[ax,ay] ; B[bx,by]
        # Point: C[cx, cy]
        # Calculate U parameter
        u = self.parameterU(A, B, C)
        if u < 0:
            distance = self.distancePoint2Point(A, C)
        elif u > 1:
            distance = self.distancePoint2Point(B, C)
        else:
            distance = self.distancePoint2Rect(A, B, C)
        return distance
        
    def parameterU(self, A, B, C):
        # Point A: [ax, ay]
        # Point B: [bx, by]
        # Point C: [cx, cy]
        # Parameter U of equations: Px = ax + u*(bx-ax); and Py = ay + u*(by-ay)
        u = ((C[0] - A[0])*(B[0] - A[0]) + (C[1] - A[1])*(B[1] - A[1])) / (pow((B[0] - A[0]),2) + pow((B[1] - A[1]),2))
        return u
        
    def distancePoint2Point(self, Point1, Point2):
        # Point: 1[x1,y1]
        # Point: 2[x2,y2]
        return math.sqrt(pow((Point2[0]-Point1[0]),2) + pow((Point2[1]-Point1[1]),2))

    def distancePoint2Rect(self, A, B, C):
        # Rect: A[ax,ay] ; B[bx,by]
        # Point: C[cx,cy]
        distance = abs((B[0] - A[0])*(C[1] - A[1]) - (B[1] - A[1])*(C[0] - A[0])) / (math.sqrt(pow((B[0]-A[0]),2) + pow((B[1]-A[1]),2)))
        return distance
    
    def distanceCar2Car(self, pointCarLeft, pointCarRight, pointFrontLeft, pointFrontRight, pointRearLeft, pointRearRight):
        # Mide la minima distancia desde los 4 vertices de un coche a la parte delantera o trasera de otro coche (segmento)
        # Segment: pointCarLeft[x,y] ; pointCarRight[x,y] 
        # Point 1: pointFrontLeft[x,y]
        # Point 2: pointFrontRight[x,y]
        # Poitn 3: pointRearLeft[x,y]
        # Point 4: pointRearRight[x,y]

        distance = self.distancePoint2Segment(pointCarLeft, pointCarRight, pointFrontLeft)

        if (self.distancePoint2Segment(pointCarLeft, pointCarRight, pointFrontRight) < distance):
            distance = self.distancePoint2Segment(pointCarLeft, pointCarRight, pointFrontRight)
        if (self.distancePoint2Segment(pointCarLeft, pointCarRight, pointRearLeft) < distance):
            distance = self.distancePoint2Segment(pointCarLeft, pointCarRight, pointRearLeft)
        if (self.distancePoint2Segment(pointCarLeft, pointCarRight, pointRearRight) < distance):
            distance = self.distancePoint2Segment(pointCarLeft, pointCarRight, pointRearRight)

        return distance


    def distances(self):
        carSize = [5.75, 2.5]
        carSizeTaxi = [4, 2]
        
        # Poses sidewalk
        positionSideWalk_start = [-25, -4.25]
        positionSideWalk_final = [35, -4.25]
        
        # Poses parked cars (origin poses)
        # Frontal car
        pointCarFrontal_RearLeft = [14 - carSize[0]/2, -3+carSize[1]/2]
        pointCarFrontal_RearRight = [14 - carSize[0]/2, -3-carSize[1]/2]
        pointCarFrontal_FrontLeft = [14 + carSize[0]/2, -3+carSize[1]/2]
        pointCarFrontal_FrontRight = [14 + carSize[0]/2, -3-carSize[1]/2]
        # Rear Car
        pointCarRear_FrontLeft = [0.5 + carSize[0]/2, -3+carSize[1]/2]
        pointCarRear_FrontRight = [0.5 + carSize[0]/2, -3-carSize[1]/2]
        pointCarRear_RearLeft = [0.5 - carSize[0]/2, -3+carSize[1]/2]
        pointCarRear_RearRight = [0.5 - carSize[0]/2, -3-carSize[1]/2]

        
        # Pose 3D (origin poses)
        xFront = self.pose3d.getX() + carSizeTaxi[0]/2
        xRear = self.pose3d.getX() - carSizeTaxi[0]/2
        yLeft = self.pose3d.getY() + carSizeTaxi[1]/2
        yRight = self.pose3d.getY() - carSizeTaxi[1]/2

        # Final poses (Car's rotation)
        pointFrontLeft = self.RTCar() * np.matrix([[xFront], [yLeft], [1], [1]])
        pointFrontLeft = [pointFrontLeft.flat[0],pointFrontLeft.flat[1]]
        pointFrontRight = self.RTCar() * np.matrix([[xFront], [yRight], [1], [1]])
        pointFrontRight = [pointFrontRight.flat[0], pointFrontRight.flat[1]]
        pointRearLeft = self.RTCar() * np.matrix([[xRear], [yLeft], [1], [1]])
        pointRearLeft = [pointRearLeft.flat[0],pointRearLeft.flat[1]]
        pointRearRight = self.RTCar() * np.matrix([[xRear], [yRight], [1], [1]])
        pointRearRight = [pointRearRight.flat[0],pointRearRight.flat[1]]
        
        # Distance car -> parked front car
        distFrontFinal_1 = self.distanceCar2Car(pointCarFrontal_RearLeft, pointCarFrontal_RearRight, pointFrontLeft, pointFrontRight, pointRearLeft, pointRearRight)
        
        # Distance parked front car -> car
        distFrontFinal_2 = self.distanceCar2Car(pointFrontLeft, pointFrontRight, pointCarFrontal_RearLeft, pointCarFrontal_RearRight, pointCarFrontal_FrontLeft , pointCarFrontal_FrontRight)
        
        # Distance car -> parked rear car
        distRearFinal_1 = self.distanceCar2Car(pointCarRear_FrontLeft, pointCarRear_FrontRight, pointFrontLeft, pointFrontRight, pointRearLeft, pointRearRight)
        
        # Distance parked rear car -> car
        distRearFinal_2 = self.distanceCar2Car(pointRearLeft, pointRearRight, pointCarRear_FrontLeft , pointCarRear_FrontRight, pointCarRear_RearLeft , pointCarRear_RearRight)

        # Minimal distance
        if distFrontFinal_1 > distFrontFinal_2:
            self.distFrontFinal = distFrontFinal_1
        else: 
            self.distFrontFinal = distFrontFinal_2
            
        if distRearFinal_1 > distRearFinal_2:
            self.distRearFinal = distRearFinal_1
        else: 
            self.distRearFinal = distRearFinal_2
        
        # Distance car -> sidewalk
        self.distanceSidewalk = self.distanceCar2Car(positionSideWalk_start, positionSideWalk_final, pointFrontLeft, pointFrontRight, pointRearLeft, pointRearRight)


    def updateG(self):
        self.distances()
        self.distanceFrontalLabel.setText("Front distance: " + str(round(self.distFrontFinal, 3)) + ' m')
        self.distanceRearLabel.setText("Rear distance: " + str(round(self.distRearFinal, 3)) + ' m')
        self.distanceSidewalkLabel.setText("Distance to the sidewalk: " + str(round(self.distanceSidewalk, 3)) + ' m')
        self.update()      
   
   
        
class markWidget(QWidget):
    def __init__(self,winParent,pose3d, time, quality, distance):    
        super(markWidget, self).__init__()
        self.winParent=winParent
        self.pose3d = pose3d
        self.time = time
        self.quality = quality
        self.distance = distance
        self.num = 0

        self.hLayout = QHBoxLayout()
        self.markLabel = ''
        
        self.button = QPushButton('Show me my mark')
        self.button.clicked.connect(self.markFinal)
        self.hLayout.addWidget(self.button, 0)
        
        self.setLayout(self.hLayout) 
        
    def markFinal(self):
        markAngle = self.testAngle() * 0.025
        markTime = self.testTime() * 0.025
        markDist = self.testDistance() * 0.025
        markCol = self.testCollision() * 0.025
        mark = markAngle + markTime + markDist + markCol
        if self.num == 0:
            self.markLabel = QLabel('Final mark: ' + str(mark))
            self.hLayout.addWidget(self.markLabel, 0)
        else:
            self.markLabel.setText('Final mark: ' + str(mark))
        self.num = self.num + 1
        
    def testAngle(self):
        yawRad = self.pose3d.getYaw()
        angle = math.degrees(yawRad) + 90
        if (angle >= 85 and angle <= 105):
            markAngle = 100
        elif (angle < 85 and angle >= 70 or angle > 105 and angle <= 120):
            markAngle = 80
        elif (angle < 70 and angle >= 60 or angle > 120 and angle <= 130):
            markAngle = 50
        else: 
            markAngle = 0
        return markAngle
    
    def testTime(self):
        minTime = 170
        myTime = self.time.seconds
        markTime = float(minTime*100)/float(myTime)
        if myTime < 170:
            markTime = 100  
        return markTime
    
    def testDistance(self):
        MyDistFront = self.distance.distFrontFinal
        MyDistRear = self.distance.distRearFinal
        MyDistSidewalk = self.distance.distanceSidewalk
        ideal = [7.25, -3]
        posX = self.pose3d.getX()
        posY = self.pose3d.getY()
        
        if MyDistFront >= 1.5 and MyDistFront < 3.5:
            markDistFront = 100
        elif MyDistFront < 1.5 and MyDistFront >= 1:
            markDistFront = 50
        else:
            markDistFront = 0

        if MyDistRear >= 1.5 and MyDistRear < 3.5:
            markDistRear = 100
        elif MyDistRear < 1.5 and MyDistRear >= 1:
            markDistRear = 50
        else:
            markDistRear = 0

        if MyDistSidewalk > 0 and MyDistSidewalk <= 0.75:
            markDistSidewalk = 100
        elif MyDistSidewalk > 0.75 and MyDistSidewalk < 1.5:
            markDistSidewalk = 50
        else:
            markDistSidewalk = 0

        markDist = float(markDistFront+markDistRear+markDistSidewalk)/float(3)
        return markDist
    
    def testCollision(self):
        minCrash = 0
        if self.quality.numCrash == 0:
            markCol = 100
        else:
            markCol = float(minCrash*100)/float(self.quality.numCrash)
        return markCol

    def updateG(self):
        self.update() 
        
             

class timeWidget(QWidget):

    time = pyqtSignal()
    def __init__(self,winParent):    
        super(timeWidget, self).__init__()
        self.winParent=winParent
        self.seconds = 0
        
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
        self.seconds += 1
        self.lcd.display(self.seconds)



class cheeseWidget(QWidget):
    
    def __init__(self,winParent, pose3d):    
        super(cheeseWidget, self).__init__()
        self.winParent=winParent
        self.rectangle = QRectF(0.0, 0.0, 300.0, 300.0)
        self.pose3d = pose3d       

    def drawRedZones(self, painter):
        self.setStyle(painter, QColor(255,70,70),QColor(255,70,70),1)
        startAngle = 0 * 16
        spanAngle = 45 * 16
        painter.drawPie(self.rectangle, startAngle, spanAngle)
        startAngle = 135 * 16
        spanAngle = 45 * 16
        painter.drawPie(self.rectangle, startAngle, spanAngle)
        startAngle = 180 * 16
        spanAngle = 180 * 16
        painter.drawPie(self.rectangle, startAngle, spanAngle)
        
    def drawOrangeZones(self, painter):
        self.setStyle(painter, QColor(255,220,23),QColor(255,220,23),1)
        startAngle = 45 * 16
        spanAngle = 30 * 16
        painter.drawPie(self.rectangle, startAngle, spanAngle)
        startAngle = 105 * 16
        spanAngle = 30 * 16
        painter.drawPie(self.rectangle, startAngle, spanAngle)

    def drawGreenZones(self, painter):
        self.setStyle(painter, QColor(117,240,154),QColor(117,240,154),1)
        startAngle = 75 * 16
        spanAngle = 15 * 16
        painter.drawPie(self.rectangle, startAngle, spanAngle)
        startAngle = 90 * 16
        spanAngle = 15 * 16
        painter.drawPie(self.rectangle, startAngle, spanAngle)

    def drawArrow(self, painter, angle=90):
        radius = 130
        yawRad = self.pose3d.getYaw()
        angle = -(yawRad + pi/2) # PI/2 para centrar la aguja
        origx = self.rectangle.width() / 2
        origy = self.rectangle.height() / 2
        finx = radius * math.cos(angle) + origx
        finy = radius * math.sin(angle) + origy   
        self.setStyle(painter, Qt.black,Qt.black,3)
        painter.drawLine(QPoint(origx,origy), QPoint(finx,finy))
        painter.drawEllipse(145,145, 10, 10)

    def resetPen(self, painter):
        pen = QPen(Qt.black, 1)
        brush = QBrush()
        painter.setPen(pen)
        painter.setBrush(brush)

    def setStyle(self, painter, fillColor, penColor, stroke):
        brush = QBrush()
        pen = QPen(penColor, stroke)
        brush.setColor(fillColor)
        brush.setStyle(Qt.SolidPattern)
        painter.setBrush(brush)
        painter.setPen(pen)
        painter.setRenderHint(QPainter.Antialiasing)
      
    def paintEvent(self, event):
        painter = QPainter(self)
        self.drawRedZones(painter)
        self.drawOrangeZones(painter)
        self.drawGreenZones(painter)
        self.drawArrow(painter,120)

    def updateG(self):
        self.update()



if __name__ == "__main__":
    
    app = QApplication(sys.argv)
    ic = EasyIce.initialize(sys.argv)
    pose3d = Pose3DClient(ic, "Autopark.Pose3D", True)
    laser1 = LaserClient(ic, "Autopark.Laser1", True)
    laser2 = LaserClient(ic, "Autopark.Laser2", True)
    laser3 = LaserClient(ic, "Autopark.Laser3", True)

    myGUI = MainWindow(pose3d, laser1, laser2, laser3)
    myGUI.show()
    t2 = ThreadGUI(myGUI)
    t2.daemon=True
    t2.start()
    sys.exit(app.exec_())
