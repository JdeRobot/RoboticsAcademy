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
from PyQt5.QtWidgets import QWidget, QLabel, QSlider, QHBoxLayout, QVBoxLayout, QGroupBox, QComboBox, QPushButton, QFrame, QCheckBox
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

        self.dropdown = QComboBox()
        self.dropdown.addItems(["None", "RGB", "HSV"])
        setattr(self.winParent.getCamera(), 'currDropdown', self.dropdown.currentText())
        self.dropdown.currentIndexChanged.connect(self.dropchange)

        self.dropdown.resize(self.dropdown.sizeHint())

        drophlayout.addWidget(self.dropdown)

        self.detectbutton = QCheckBox("Detector")
        self.detectbutton.toggled.connect(self.detectcheck)

        drophlayout.addSpacing(250)
        drophlayout.addWidget(self.detectbutton)

        vlayout.addLayout(drophlayout)

        
        for color in ["HSV", "RGB"]:
            scolor = color.lower()
            setattr(self, scolor+"frame", QFrame(self))
            setattr(self, scolor+"hlayout", QHBoxLayout())
            setattr(self, scolor+"groupbox", QGroupBox())

            getattr(self, scolor+"groupbox").setLayout(self.givecolorlayout(color))
            getattr(self, scolor+"hlayout").addWidget(getattr(self, scolor+"groupbox"))
            
            button = QPushButton("Reset", self)
            button.clicked.connect(self.resetwrapper(color))
            getattr(self, scolor+"hlayout").addWidget(button)
            getattr(self, scolor+"frame").setLayout(getattr(self, scolor+"hlayout"))
            vlayout.addWidget(getattr(self, scolor+"frame"))
            getattr(self, scolor+"frame").hide()


    def detectcheck(self):
        if self.detectbutton.isChecked() == True:
            self.detectorGUI.show()
        else:
            self.detectorGUI.hide()


    def givecolorlayout(self, color):
        rvlayout = QVBoxLayout()
        for c in color:
            for mm in ["min", "max"]:
                rvlayout.addLayout(self.createSlider(color + "_" + c + "_" + mm))

        return rvlayout

    def resetwrapper(self, name):
        mymin = "0" #if "RGB" in name else (if )"0"
        def lambdafunc():
            for space in name[:3]:
                for mm in ["min", "max"]:
                    getattr(self, "%s_slider"%(name+"_"+space+"_"+mm)).setValue(int(mymin)) ; 
                    getattr(self, "%s_out"%(name+"_"+space+"_"+mm)).setText(mymin) ; 
                    getattr(self.winParent.getCamera(), "%s_out"%(name+"_"+space+"_"+mm), int(mymin))
        return lambdafunc


    def dropchange(self, index):
        currDropdown = self.dropdown.currentText()
        setattr(self.winParent.getCamera(), 'currDropdown', self.dropdown.currentText())
        self.rgbframe.hide()
        self.hsvframe.hide()
        if currDropdown != "None":
            getattr(self, "%sframe"%(currDropdown.lower())).show()

    def createSlider(self, name):
        mymin = "0" #if "RGB" in name else (if )"0"
        mymax = "255" if "RGB" in name else ("179" if ("_H_" in name and "HSV" in name) else ("255" if "HSV" in name else "0"))

        hlayout = QHBoxLayout()
        namewidget = QLabel(name)
        hlayout.addWidget(namewidget)
        setattr(self, "%s_slider"%(name), QSlider(Qt.Horizontal))

        getattr(self, "%s_slider"%(name)).setMinimum(int(mymin))
        getattr(self, "%s_slider"%(name)).setMaximum(int(mymax))
        getattr(self, "%s_slider"%(name)).setTickPosition(QSlider.TicksBelow)
        getattr(self, "%s_slider"%(name)).setTickInterval(20)
        getattr(self, "%s_slider"%(name)).setValue(int(mymin))

        getattr(self, "%s_slider"%(name)).valueChanged.connect(self.funcwrapper(name))   
        hlayout.addWidget(getattr(self, "%s_slider"%(name)))

        setattr(self, "%s_out"%(name), QLabel(mymin))
        setattr(self.winParent.getCamera(), "%s_out"%(name), int(mymin))
        hlayout.addWidget(getattr(self, "%s_out"%(name)))

        return hlayout
        
    def funcwrapper(self, name):
        def lmbdafunc(): 
            getattr(self, "%s_out"%(name)).setText(str(getattr(self, "%s_slider"%(name)).value()))
            setattr(self.winParent.getCamera(), "%s_out"%(name), getattr(self, "%s_slider"%(name)).value())
        return lmbdafunc


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