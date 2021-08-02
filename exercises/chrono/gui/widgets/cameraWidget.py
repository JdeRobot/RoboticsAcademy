
from PyQt5 import QtGui,QtCore
import cv2


class CameraWidget:
    IMG_WIDTH=400
    IMG_HEIGHT=300

    def __init__(self,winParent):
        self.winParent=winParent
        self.labelImage=winParent.image
        self.labelImageFiltered = winParent.imageFiltered


    def updateImage(self):

        img = self.winParent.getCamera().getImage().data
        if img is not  None:
            resized = cv2.resize(img,(self.IMG_WIDTH,self.IMG_HEIGHT))
            image = QtGui.QImage(resized.data, resized.shape[1], resized.shape[0], resized.shape[1]*resized.shape[2], QtGui.QImage.Format_RGB888);
            size=QtCore.QSize(img.shape[1],img.shape[0])
            # self.labelImage.resize(size)
            self.labelImage.setPixmap(QtGui.QPixmap.fromImage(image))

        #print the filtered images

        imgFiltered = self.winParent.getAlgorithm().getImageFiltered()
        if imgFiltered is not None:
            resized = cv2.resize(imgFiltered,(self.IMG_WIDTH,self.IMG_HEIGHT))
            image = QtGui.QImage(resized.data, resized.shape[1], resized.shape[0], resized.shape[1]*resized.shape[2], QtGui.QImage.Format_RGB888);
            size=QtCore.QSize(imgFiltered.shape[1],imgFiltered.shape[0])
            # self.labelImageFiltered.resize(size)
            self.labelImageFiltered.setPixmap(QtGui.QPixmap.fromImage(image))
