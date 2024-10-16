import numpy as np
import math
from math import pi as pi
import cv2
import matplotlib.pyplot as plt

class Map:
	def __init__(self, pose3d):
		# The world scenario is placed on 0,0. It has a size of 500x500
		# The coordinates of the map are from 0 to 400, being 0,0 the top left corner
		self.pose3d = pose3d
		self.worldWidth = 500
		self.worldHeight = 500
		self.gridWidth = 400
		self.gridHeight = 400
		self.grid = np.empty([self.gridWidth, self.gridHeight], float)
	
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

	def RTWorldGrid(self):
		RTx = self.RTx(pi, 0, 0, 0)
		RTz = self.RTz(-pi/2, self.gridWidth/2, -self.gridHeight/2, 0)
		return RTx*RTz

	def RTGridWorld(self):
		RTx = self.RTx(-pi, 0, 0, 0)
		RTz = self.RTz(pi/2, self.worldWidth/2, self.worldHeight/2, 0)
		return RTz*RTx

	def worldToGrid(self, worldX, worldY):
		""" Transform a world point to map cell coordinates """
		gridX = (worldY + self.worldHeight/2) * self.gridHeight / self.worldHeight
		gridY = (worldX + self.worldWidth/2) * self.gridWidth / self.worldWidth
		gridX = int(np.clip(gridX, 0, self.gridWidth))
		gridY = int(np.clip(gridY, 0, self.gridHeight))
		return (gridX, gridY)

	def gridToWorld(self, gridX, gridY):
		""" Transform a map cell to world coordinates """
		worldX = gridY * self.worldHeight / self.gridHeight - self.worldHeight/2
		worldY = gridX * self.worldWidth / self.gridWidth - self.worldWidth/2
		return (worldX, worldY)

	def RTFormula(self):
		RTz = self.RTz(pi/2, 50, 70, 0)
		return RTz

	def getGridVal(self, x, y):
		tmp = self.grid[y][x]
		return tmp

	def setGridVal(self, x, y, val):
		self.grid[y][x] = val

	def getTaxiCoordinates(self):
		""" Return the taxi pose in map coordinates """
		pose = self.pose3d()
		return self.worldToGrid(pose.x, pose.y)

	def rowColumn(self, pose):
		# Deprecated. Use worldToGrid()
		x = pose[0]
		y = pose[1]
		#print("x : {} , y : {}".format(x,y))
		# Transform from world coordinates to map coordinates
		# The scenario is placed on 0,0. It has a length of 500x500
		# The coordinates of the map are from 0 to 400, being 0,0 the top left corner
		x = x + 250
		x = x / 500
		x = x * 400
		if (x > 400):
			x = 400
		if (x < 0):
			x = 0
		x = int(x)
		y = y + 250
		y = y / 500
		y = y * 400
		if (y > 400):
			y = 400
		if (y < 0):
			y = 0
		y = int(y)
		#print("x : {} , y : {}".format(x,y))
		return [y, x]

	def getTaxiAngle(self):
		pose = self.pose3d()
		rt = pose.yaw 
		#print(rt)
		ty = math.cos(-rt) - math.sin(-rt)
		tx = math.sin(-rt) + math.cos(-rt)
		#print("tx : {} , ty : {}".format(tx,ty))
		return rt,

	# Function to reset
	def reset(self):
		# Nothing to do, service takes care!
		pass

	def robotPose(self):
		pass

	def getMap(self, url):
		return cv2.imread(url, cv2.IMREAD_GRAYSCALE)