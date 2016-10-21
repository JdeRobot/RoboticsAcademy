
from PyQt5.QtCore import QSize
from PyQt5.QtGui import QImage, QPixmap
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
            image = QImage(resized.data, resized.shape[1], resized.shape[0], resized.shape[1]*resized.shape[2], QImage.Format_RGB888);
            size=QSize(imgLeft.shape[1],imgLeft.shape[0])
            #self.label.resize(size)
            self.labelImageLeft.setPixmap(QPixmap.fromImage(image))

        imgRight = self.winParent.getSensor().getImageRight()
        if imgRight != None:
            resized = cv2.resize(imgRight,(self.IMG_WIDTH,self.IMG_HEIGHT))
            image = QImage(resized.data, resized.shape[1], resized.shape[0], resized.shape[1]*resized.shape[2], QImage.Format_RGB888);
            size=QSize(imgRight.shape[1],imgRight.shape[0])
            #self.label.resize(size)
            self.labelImageRight.setPixmap(QPixmap.fromImage(image))


        #print the filtered images

        imgLeftFiltered = self.winParent.getAlgorithm().getLeftImageFiltered()
        if imgLeftFiltered != None:
            resized = cv2.resize(imgLeftFiltered,(self.IMG_WIDTH,self.IMG_HEIGHT))
            image = QImage(resized.data, resized.shape[1], resized.shape[0], resized.shape[1]*resized.shape[2], QImage.Format_RGB888);
            size=QSize(imgLeftFiltered.shape[1],imgLeftFiltered.shape[0])
            #self.label.resize(size)
            self.labelImageLeftFiltered.setPixmap(QPixmap.fromImage(image))

        imgRightFiltered = self.winParent.getAlgorithm().getRightImageFiltered()
        if imgRightFiltered != None:
            resized = cv2.resize(imgRightFiltered,(self.IMG_WIDTH,self.IMG_HEIGHT))
            image = QImage(resized.data, resized.shape[1], resized.shape[0], resized.shape[1]*resized.shape[2], QImage.Format_RGB888);
            size=QSize(imgRightFiltered.shape[1],imgRightFiltered.shape[0])
            #self.label.resize(size)
            self.labelImageRightFiltered.setPixmap(QPixmap.fromImage(image))
