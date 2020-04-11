from PyQt5 import QtGui,QtCore
import cv2

class CameraWidget:
    IMG_WIDTH=320
    IMG_HEIGHT=240
    def __init__(self,winParent):
        self.winParent=winParent
        self.labelImage=winParent.orig_img
        self.labelImageFiltered = winParent.processed_img

    def updateImage(self):
    	img = self.winParent.getRGBImage()
    	if img is not None:
    		resized = cv2.resize(img,(self.IMG_WIDTH,self.IMG_HEIGHT))
    		image = QtGui.QImage(resized.data, resized.shape[1], resized.shape[0], resized.shape[1]*resized.shape[2], QtGui.QImage.Format_RGB888)
    		self.labelImage.setPixmap(QtGui.QPixmap.fromImage(image))

    	processed_img = self.winParent.getAlgorithm().get_processed_image()
    	if processed_img is not None:
            resized = cv2.resize(processed_img,(self.IMG_WIDTH,self.IMG_HEIGHT))
            image = QtGui.QImage(resized.data, resized.shape[1], resized.shape[0], resized.shape[1]*resized.shape[2], QtGui.QImage.Format_RGB888)
            self.labelImageFiltered.setPixmap(QtGui.QPixmap.fromImage(image))