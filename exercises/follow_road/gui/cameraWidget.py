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
        self.winParent = winParent
        self.labelImageVentral = winParent.imageVentral
        self.labelImageFrontal = winParent.imageFrontal
        self.labelImageFilteredVentral = winParent.imageFilteredVentral
        self.labelImageFilteredFrontal = winParent.imageFilteredFrontal
        self.initUI()

    def initUI(self):

        self.setMinimumSize(680,500)
        self.setMaximumSize(680,500)

        self.imgLabel=QLabel(self)
        self.imgLabel.resize(640,360)
        self.imgLabel.move(10,5)
        self.imgLabel.show()

    def updateImage(self):

        imgV = self.winParent.getDrone().getImageVentral().data
        if imgV is not  None:
            resized = cv2.resize(imgV,(self.IMG_WIDTH,self.IMG_HEIGHT))
            imageV = QtGui.QImage(resized.data, resized.shape[1], resized.shape[0], resized.shape[1]*resized.shape[2], QtGui.QImage.Format_RGB888);
            size=QtCore.QSize(imgV.shape[1],imgV.shape[0])
            #self.label.resize(size)
            self.labelImageVentral.setPixmap(QtGui.QPixmap.fromImage(imageV))

        imgF = self.winParent.getDrone().getImageFrontal().data
        if imgF is not  None:
            resized = cv2.resize(imgF,(self.IMG_WIDTH,self.IMG_HEIGHT))
            imageF = QtGui.QImage(resized.data, resized.shape[1], resized.shape[0], resized.shape[1]*resized.shape[2], QtGui.QImage.Format_RGB888);
            size=QtCore.QSize(imgF.shape[1],imgF.shape[0])
            #self.label.resize(size)
            self.labelImageFrontal.setPixmap(QtGui.QPixmap.fromImage(imageF))

        #print the filtered images

        imgFilteredV = self.winParent.getAlgorithm().getImageFilteredVentral()
        if imgFilteredV is not None:
            resized = cv2.resize(imgFilteredV,(self.IMG_WIDTH,self.IMG_HEIGHT))
            imageV = QtGui.QImage(resized.data, resized.shape[1], resized.shape[0], resized.shape[1]*resized.shape[2], QtGui.QImage.Format_RGB888);
            size=QtCore.QSize(imgFilteredV.shape[1],imgFilteredV.shape[0])
            #self.label.resize(size)
            self.labelImageFilteredVentral.setPixmap(QtGui.QPixmap.fromImage(imageV))

        imgFilteredF = self.winParent.getAlgorithm().getImageFilteredFrontal()
        if imgFilteredF is not None:
            resized = cv2.resize(imgFilteredF,(self.IMG_WIDTH,self.IMG_HEIGHT))
            imageF = QtGui.QImage(resized.data, resized.shape[1], resized.shape[0], resized.shape[1]*resized.shape[2], QtGui.QImage.Format_RGB888);
            size=QtCore.QSize(imgFilteredF.shape[1],imgFilteredF.shape[0])
            #self.label.resize(size)
            self.labelImageFilteredFrontal.setPixmap(QtGui.QPixmap.fromImage(imageF))

    def closeEvent(self, event):
        self.winParent.closeCameraWidget()
