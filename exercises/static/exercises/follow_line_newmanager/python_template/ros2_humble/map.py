import numpy as np
import math
from math import pi as pi
import cv2

class Map:
	def __init__(self, pose3d, circuit):
		self.pose3d = pose3d
		self.circuit = circuit
	
	def RTx(self, angle, tx, ty, tz):
		RT = np.matrix([[1, 0, 0, tx], [0, math.cos(angle), -math.sin(angle), ty], 
						[0, math.sin(angle), math.cos(angle), tz], [0, 0, 0, 1]])
		return RT
		
	def RTy(self, angle, tx, ty, tz):
		RT = np.matrix([[math.cos(angle), 0, math.sin(angle), tx], [0, 1, 0, ty],
						[-math.sin(angle), 0, math.cos(angle), tz], [0, 0, 0, 1]])
		return RT
		
	def RTz(self, angle, tx, ty, tz):
		RT = np.matrix([[math.cos(angle), -math.sin(angle), 0, tx], [math.sin(angle), math.cos(angle), 0, ty],
						[0, 0, 1, tz], [0, 0, 0, 1]])
		return RT
		
	def RTFormula(self):
		RTz = self.RTz(pi/2, 50, 70, 0)
		return RTz
		
	def getFormulaCoordinates(self):
		
		pose = self.pose3d.getPose3d()
		x = pose.x
		y = pose.y
		
		if self.circuit == "default":
			# Default
			scale_y = 1.25; offset_y = 77
			scale_x = -2.6; offset_x = 151
		elif self.circuit == "montmelo":
			# Montmelo
			scale_y = 2.1; offset_y = 77
			scale_x = -1.3; offset_x = 151
		elif self.circuit == "montreal":
			# Montreal
			#scale_y = 0.6; offset_y = 76
			scale_y = 0.685; offset_y = 77
			scale_x = -0.48; offset_x = 151
		elif self.circuit == "nbg":
			scale_y = 1.5; offset_y = 77
			scale_x = -1.495; offset_x = 151
		
		
		y = scale_y * y + offset_y
		x = scale_x * x + offset_x
		
		
		# Default
		#scale_x = -2.6; offset_x = 151
		# Montmelo
		#scale_x = -1.3; offset_x = 151
		# Montreal
		#scale_x = -0.48; offset_x = 151

		return x, y

	# Function to reset
	def reset(self):
		# Nothing to do, service takes care!
		pass
	
