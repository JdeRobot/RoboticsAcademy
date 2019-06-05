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
from PyQt5.QtCore import QSize, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QPushButton,QWidget, QLabel
from PyQt5 import QtGui,QtCore
import cv2

class CameraWidget(QWidget):
    IMG_WIDTH=320
    IMG_HEIGHT=240

    imageUpdate=pyqtSignal()

    def __init__(self,winParent):
        super(CameraWidget, self).__init__()
        self.winParent=winParent
        self.labelImage=winParent.image
        self.labelImageFiltered = winParent.imageFiltered
        self.initUI()

    def initUI(self):

        self.setMinimumSize(680,500)
        self.setMaximumSize(680,500)

        self.imgLabel=QLabel(self)
        self.imgLabel.resize(640,360)
        self.imgLabel.move(10,5)
        self.imgLabel.show()

    def updateImage(self):

        img = self.winParent.getDrone().getImageFrontal().data
        if img is not  None:
            resized = cv2.resize(img,(self.IMG_WIDTH,self.IMG_HEIGHT))
            image = QtGui.QImage(resized.data, resized.shape[1], resized.shape[0], resized.shape[1]*resized.shape[2], QtGui.QImage.Format_RGB888);
            size=QtCore.QSize(img.shape[1],img.shape[0])
            #self.label.resize(size)
            self.labelImage.setPixmap(QtGui.QPixmap.fromImage(image))

        #print the filtered images

        imgFiltered = self.winParent.getAlgorithm().getImageFiltered()
        if imgFiltered is not None:
            resized = cv2.resize(imgFiltered,(self.IMG_WIDTH,self.IMG_HEIGHT))
            image = QtGui.QImage(resized.data, resized.shape[1], resized.shape[0], resized.shape[1]*resized.shape[2], QtGui.QImage.Format_RGB888);
            size=QtCore.QSize(imgFiltered.shape[1],imgFiltered.shape[0])
            #self.label.resize(size)
            self.labelImageFiltered.setPixmap(QtGui.QPixmap.fromImage(image))

    def closeEvent(self, event):
        self.winParent.closeCameraWidget()
