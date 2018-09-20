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

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget, QLabel, QPushButton
from PyQt5.QtGui import QImage, QPixmap

class ColorFilterWidget(QWidget):
    IMAGE_COLS_MAX=640
    IMAGE_ROWS_MAX=360
    INITIAL_COLOR_X=20
    INITIAL_COLOR_Y=20
    INITIAL_FILT_X= 40 + 640 #IMAGE_COLS_MAX
    INITIAL_FILT_Y=20
    WINDOW_X = 1340
    WINDOW_Y = 500
    
    imageUpdate=pyqtSignal()
    
    def __init__(self,winParent):      
        super(ColorFilterWidget, self).__init__()
        self.winParent=winParent
        self.imageUpdate.connect(self.updateImage)
        self.initUI()
        
    def initUI(self):

        self.setWindowTitle("Color filter")

        self.setMinimumSize(self.WINDOW_X,self.WINDOW_Y)
        self.setMaximumSize(self.WINDOW_X,self.WINDOW_Y)

        self.imgLabelColor=QLabel(self)
        self.imgLabelColor.resize(self.IMAGE_COLS_MAX,self.IMAGE_ROWS_MAX)
        self.imgLabelColor.move(self.INITIAL_COLOR_X,self.INITIAL_COLOR_Y)
        self.imgLabelColor.show()

        self.imgLabelFiltered=QLabel(self)
        self.imgLabelFiltered.resize(self.IMAGE_COLS_MAX,self.IMAGE_ROWS_MAX)
        self.imgLabelFiltered.move(self.INITIAL_FILT_X,self.INITIAL_FILT_Y)
        self.imgLabelFiltered.show()

        self.setWindowTitle("Camera")
        changeCamButton=QPushButton("Change Camera")
        changeCamButton.resize(170,40)
        changeCamButton.move(595,450)
        changeCamButton.setParent(self)
        changeCamButton.clicked.connect(self.changeCamera)

    def setColorImage(self):
        img = self.winParent.getDrone().getImage().data

        if img is not None:
            image = QImage(img.data, img.shape[1], img.shape[0], img.shape[1] * img.shape[2], QImage.Format_RGB888)
            self.imgLabelColor.setPixmap(QPixmap.fromImage(image))
            if img.shape[1]==self.IMAGE_COLS_MAX:
                x=self.INITIAL_COLOR_X
            else:
                x=(self.IMAGE_COLS_MAX+self.INITIAL_COLOR_X)/2-(img.shape[1]/2)
            if img.shape[0]==self.IMAGE_ROWS_MAX:
                y=self.INITIAL_COLOR_Y
            else:
                y=(self.IMAGE_ROWS_MAX+self.INITIAL_COLOR_Y)/2-(img.shape[0]/2)
            
            self.imgLabelColor.move(x,y)

    def setFilteredImage(self):
        img = self.winParent.getAlgorithm().getFilteredImage()

        if img is not None:
            image = QImage(img.data, img.shape[1], img.shape[0], img.shape[1] * img.shape[2], QImage.Format_RGB888)
            self.imgLabelFiltered.setPixmap(QPixmap.fromImage(image))

            if img.shape[1]==self.IMAGE_COLS_MAX:
                x=self.INITIAL_FILT_X
            else:
                x=(self.IMAGE_COLS_MAX+self.INITIAL_COLOR_X)/2-(img.shape[1]/2) + self.WINDOW_X/2
            if img.shape[0]==self.IMAGE_ROWS_MAX:
                y=self.INITIAL_FILT_Y
            else:
                y=(self.IMAGE_ROWS_MAX+self.INITIAL_COLOR_X)/2-(img.shape[0]/2)
            
            self.imgLabelFiltered.move(x,y)
        
    def updateImage(self):
        self.setColorImage()
        self.setFilteredImage()
        
    def closeEvent(self, event):
        self.winParent.closeCameraWidget()

    def changeCamera(self):
        self.winParent.getDrone().toggleCam()
