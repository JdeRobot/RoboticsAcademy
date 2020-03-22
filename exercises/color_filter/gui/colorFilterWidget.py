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
import cv2
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtWidgets import QWidget, QLabel, QHBoxLayout, QVBoxLayout, QGroupBox, QCheckBox
from PyQt5.QtGui import QImage, QPixmap
from gui.detectorWidget import DetectorWidget

class ColorFilterWidget(QWidget):    
    imageUpdate=pyqtSignal()
    
    def __init__(self,winParent):      
        super(ColorFilterWidget, self).__init__()
        self.winParent=winParent
        self.imageUpdate.connect(self.updateImage)
        self.initUI()
        self.detectorGUI = DetectorWidget(self)
        self.detectorGUI.hide()

    def initUI(self):

        self.setWindowTitle("Color filter")

        vlayout = QVBoxLayout(self)


        groupbox = QGroupBox()
        hlayout = QHBoxLayout()
        hlayout.setAlignment(Qt.AlignCenter)

        self.imgLabelColor = QLabel()
        self.imgLabelColor.setFixedSize(640, 360)
        
        hlayout.addWidget(self.imgLabelColor)

        #hlayout.addStretch()

        self.imgLabelBlackWhite = QLabel()
        self.imgLabelBlackWhite.setFixedSize(640, 360)

        hlayout.addWidget(self.imgLabelBlackWhite)
        
        groupbox.setLayout(hlayout)
        vlayout.addWidget(groupbox)
        vlayout.addSpacing(25)


        drophlayout = QHBoxLayout()

        drophlayout.setAlignment(Qt.AlignCenter)
        self.detectbutton = QCheckBox("Detector")
        self.detectbutton.toggled.connect(self.detectcheck)

        drophlayout.addWidget(self.detectbutton)

        vlayout.addLayout(drophlayout)


    def detectcheck(self):
        if self.detectbutton.isChecked() == True:
            self.detectorGUI.show()
        else:
            self.detectorGUI.hide()

    def setColorImage(self):
        img = self.winParent.getCamera().getColorImage()
        if img is not None:
            image = QImage(img.data, img.shape[1], img.shape[0], img.shape[1] * img.shape[2], QImage.Format_RGB888)
            self.imgLabelColor.setPixmap(QPixmap.fromImage(image))

    def setThresholdImage(self):
        img = self.winParent.getCamera().getThresholdImage()
        if img is not None:
            image = QImage(img.data, img.shape[1], img.shape[0], img.shape[1] * img.shape[2], QImage.Format_RGB888)
            self.imgLabelBlackWhite.setPixmap(QPixmap.fromImage(image))
        
    def updateImage(self):
        self.setColorImage()
        self.setThresholdImage()
        self.detectorGUI.imageUpdate.emit()

    def closeEvent(self, event):
        self.closeDetectorWidget()
        self.winParent.closeColorFilterWidget()

    def closeDetectorWidget(self):
        self.detectbutton.setChecked(False)