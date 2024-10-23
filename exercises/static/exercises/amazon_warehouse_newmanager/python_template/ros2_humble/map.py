import numpy as np
import math
from math import pi as pi
import cv2

class Map:
	def __init__(self, pose3d):
		self.pose3d = pose3d
	
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
		
	def RTVacuum(self):
		RTz = self.RTz(pi/2, 50, 70, 0)
		return RTz
		
	def getRobotCoordinates(self):
		pose = self.pose3d()

		x = pose.x
		y = pose.y

		x = (6.8 - x) * 20.22 * 0.545
		y = (10.31 - y) * 20.17 * 0.72
		# print(" - Coordinate: " + str(x) + ", " + str(y))

		return y, x
	
	def getRobotAngle(self):
		pose = self.pose3d()
		rt = pose.yaw - 1.24

		ty = math.cos(-rt) - math.sin(-rt)
		tx = math.sin(-rt) + math.cos(-rt)

		# print(" - Angle: " + str(tx) + ", " + str(ty))
		# print(" -> rt: " + str(rt))

		return rt,

	# Function to reset
	def reset(self):
		# Nothing to do, service takes care!
		pass
	
