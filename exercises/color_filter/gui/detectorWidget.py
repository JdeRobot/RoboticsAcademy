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
from PyQt5.QtWidgets import QWidget, QLabel
from PyQt5.QtGui import QImage, QPixmap

class DetectorWidget(QWidget):
	imageUpdate=pyqtSignal()

	def __init__(self, winParent):
		super(DetectorWidget, self).__init__()
		self.winParent = winParent
		self.imageUpdate.connect(self.updateImage)

		self.setWindowTitle("Detector")

		self.imgDetect = QLabel(self)
		self.imgDetect.setFixedSize(640, 360)

	def setDetectImage(self):
		img = self.winParent.winParent.getCamera().getDetectImage()
		if img is not None:
			image = QImage(img.data, img.shape[1], img.shape[0], img.shape[1] * img.shape[2], QImage.Format_RGB888)
			self.imgDetect.setPixmap(QPixmap.fromImage(image))

	def updateImage(self):
		self.setDetectImage()

	def closeEvent(self, event):
	    self.winParent.closeDetectorWidget()
