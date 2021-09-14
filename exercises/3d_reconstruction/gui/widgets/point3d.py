__author__ = 'frivas'
from PyQt5.QtGui import QStandardItem 
from PyQt5.QtWidgets import QMessageBox, QListWidgetItem


class Point3D(QListWidgetItem):
	"""Class representing traditional 3d point as three floating point numbers"""
	def __init__(self,xpos=0.0,ypos=0.0,zpos=0.0):
		QStandardItem.__init__(self,"X: "+str(xpos)+" Y: "+str(ypos)+" Z: "+str(zpos))
		try:
			self.x=float(xpos)
			self.y=float(ypos)
			self.z=float(zpos)
		except ValueError:
			#raise
			raise ValueError("could not set point values")
			#QMessageBox.critical(None, "Point Error",
			#			"Error updating point, point value not a real number.")
		#self.marker=None

	def __str__(self):
		return "X: "+str(self.x)+" Y: "+str(self.y)+" Z: "+str(self.z)
	def __repr__(self):
		return str(self)
	#marker not used, currently handled by path_point_manager to prevent tight coupling
	def set_marker(self, marker):
		"""function links point in path to marker"""
		self.marker=marker

	def update_coords(self, c1, c2, c3):
		try:
			self.x=float(c1)
			self.y=float(c2)
			self.z=float(c3)
			self.setText(str(self))
		except ValueError:
			raise ValueError("could not set point values")
			#QMessageBox.critical(None, "Point Error",
      #      "Error updating point, point value not a real number.")