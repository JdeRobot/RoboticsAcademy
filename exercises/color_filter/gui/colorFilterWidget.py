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

from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtWidgets import QWidget, QLabel, QSlider, QHBoxLayout, QVBoxLayout, QGroupBox, QComboBox
from PyQt5.QtGui import QImage, QPixmap

class ColorFilterWidget(QWidget):
    IMAGE_COLS_MAX=640
    IMAGE_ROWS_MAX=360
    
    imageUpdate=pyqtSignal()
    
    def __init__(self,winParent):      
        super(ColorFilterWidget, self).__init__()
        self.winParent=winParent
        self.imageUpdate.connect(self.updateImage)
        self.initUI()
        
    def initUI(self):

        self.setWindowTitle("Color filter")

        vlayout = QVBoxLayout(self)


        groupbox = QGroupBox()
        hlayout = QHBoxLayout()

        self.imgLabelColor=QLabel()
        self.imgLabelColor.setFixedSize(640, 360)
        
        hlayout.addWidget(self.imgLabelColor)

        #hlayout.addStretch()

        self.imgLabelBlackWhite=QLabel()
        self.imgLabelBlackWhite.setFixedSize(640, 360)

        hlayout.addWidget(self.imgLabelBlackWhite)
        
        groupbox.setLayout(hlayout)
        vlayout.addWidget(groupbox)
        vlayout.addSpacing(25)

        self.dropdown = QComboBox()
        self.dropdown.addItems(["None", "RGB", "HSV"])
        setattr(self.winParent.getCamera(), 'currDropdown', self.dropdown.currentText())
        self.dropdown.currentIndexChanged.connect(self.dropchange)

        vlayout.addWidget(self.dropdown)

        self.rgbgroupbox = QGroupBox()
        rvlayout = QVBoxLayout(self)

        rvlayout.addLayout(self.createSlider("RGB_R_min"))
        rvlayout.addLayout(self.createSlider("RGB_G_min"))
        rvlayout.addLayout(self.createSlider("RGB_B_min"))
        rvlayout.addLayout(self.createSlider("RGB_R_max"))
        rvlayout.addLayout(self.createSlider("RGB_G_max"))
        rvlayout.addLayout(self.createSlider("RGB_B_max"))

        self.rgbgroupbox.setLayout(rvlayout)
        vlayout.addWidget(self.rgbgroupbox)


        self.hsvgroupbox = QGroupBox()
        rvlayout = QVBoxLayout(self)

        rvlayout.addLayout(self.createSlider("HSV_H_min"))
        rvlayout.addLayout(self.createSlider("HSV_S_min"))
        rvlayout.addLayout(self.createSlider("HSV_V_min"))
        rvlayout.addLayout(self.createSlider("HSV_H_max"))
        rvlayout.addLayout(self.createSlider("HSV_S_max"))
        rvlayout.addLayout(self.createSlider("HSV_V_max"))

        self.hsvgroupbox.setLayout(rvlayout)
        vlayout.addWidget(self.hsvgroupbox)

        self.rgbgroupbox.hide()
        self.hsvgroupbox.hide()



    def dropchange(self, index):
        currDropdown = self.dropdown.currentText()
        setattr(self.winParent.getCamera(), 'currDropdown', self.dropdown.currentText())
        self.rgbgroupbox.hide()
        self.hsvgroupbox.hide()
        if currDropdown != "None":
            getattr(self, "%sgroupbox"%(currDropdown.lower())).show()

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
        getattr(self, "%s_slider"%(name)).setValue(int(mymin) if "min" in name else int(mymax))


        getattr(self, "%s_slider"%(name)).valueChanged.connect(self.funcwrapper(name))   
        hlayout.addWidget(getattr(self, "%s_slider"%(name)))

        setattr(self, "%s_out"%(name), QLabel(mymin) if "min" in name else QLabel(mymax))
        setattr(self.winParent.getCamera(), "%s_out"%(name), mymin if "min" in name else mymax)
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
            image = QImage(img.data, img.shape[1], img.shape[0], img.shape[1], QImage.Format_Grayscale8)
            self.imgLabelBlackWhite.setPixmap(QPixmap.fromImage(image))
        
    def updateImage(self):
        self.setColorImage()
        self.setThresholdImage()
        
    def closeEvent(self, event):
        self.winParent.closeColorFilterWidget()
