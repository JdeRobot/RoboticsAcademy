#
#  Copyright (C) 1997-2016 JDE Developers Team
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

from PyQt4 import QtGui,QtCore
import cv2

class CameraWidget:
    IMG_WIDTH=320
    IMG_HEIGHT=240

    def __init__(self,winParent):
        self.winParent=winParent
        self.labelImageLeft=winParent.imageLeft
        self.labelImageRight=winParent.imageRight
        self.labelImageRightFiltered = winParent.imageRightFiltered
        self.labelImageLeftFiltered = winParent.imageLeftFiltered


    def updateImage(self):

        imgLeft = self.winParent.getSensor().getImageLeft()
        if imgLeft != None:
            resized = cv2.resize(imgLeft,(self.IMG_WIDTH,self.IMG_HEIGHT))
            image = QtGui.QImage(resized.data, resized.shape[1], resized.shape[0], resized.shape[1]*resized.shape[2], QtGui.QImage.Format_RGB888);
            size=QtCore.QSize(imgLeft.shape[1],imgLeft.shape[0])
            #self.label.resize(size)
            self.labelImageLeft.setPixmap(QtGui.QPixmap.fromImage(image))

        imgRight = self.winParent.getSensor().getImageRight()
        if imgRight != None:
            resized = cv2.resize(imgRight,(self.IMG_WIDTH,self.IMG_HEIGHT))
            image = QtGui.QImage(resized.data, resized.shape[1], resized.shape[0], resized.shape[1]*resized.shape[2], QtGui.QImage.Format_RGB888);
            size=QtCore.QSize(imgRight.shape[1],imgRight.shape[0])
            #self.label.resize(size)
            self.labelImageRight.setPixmap(QtGui.QPixmap.fromImage(image))


        #print the filtered images

        imgLeftFiltered = self.winParent.getAlgorithm().getLeftImageFiltered()
        if imgLeftFiltered != None:
            resized = cv2.resize(imgLeftFiltered,(self.IMG_WIDTH,self.IMG_HEIGHT))
            image = QtGui.QImage(resized.data, resized.shape[1], resized.shape[0], resized.shape[1]*resized.shape[2], QtGui.QImage.Format_RGB888);
            size=QtCore.QSize(imgLeftFiltered.shape[1],imgLeftFiltered.shape[0])
            #self.label.resize(size)
            self.labelImageLeftFiltered.setPixmap(QtGui.QPixmap.fromImage(image))

        imgRightFiltered = self.winParent.getAlgorithm().getRightImageFiltered()
        if imgRightFiltered != None:
            resized = cv2.resize(imgRightFiltered,(self.IMG_WIDTH,self.IMG_HEIGHT))
            image = QtGui.QImage(resized.data, resized.shape[1], resized.shape[0], resized.shape[1]*resized.shape[2], QtGui.QImage.Format_RGB888);
            size=QtCore.QSize(imgRightFiltered.shape[1],imgRightFiltered.shape[0])
            #self.label.resize(size)
            self.labelImageRightFiltered.setPixmap(QtGui.QPixmap.fromImage(image))
