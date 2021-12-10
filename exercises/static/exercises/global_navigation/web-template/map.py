import numpy as np
import math
from math import pi as pi
import cv2

class MAP:
	def __init__(self, pose3d):
		self.pose3d = pose3d
		self.worldWidth = 500
		self.worldHeight = 500
		self.gridWidth = 450
		self.gridHeight = 440
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
		# self.gWidth/self.wWidth is the scale
		worldX = worldX * self.gridWidth/self.worldWidth
		worldY = worldY * self.gridHeight/self.worldHeight
		orig_poses = np.matrix([[worldX], [worldY], [0], [1]])
		final_poses = self.RTWorldGrid() * orig_poses
		gridX = int(final_poses.flat[0])
		gridY = int(final_poses.flat[1])
		return (gridX, gridY)

	def gridToWorld(self, gridX, gridY):
		# self.wWidth/self.gWidth is the scale
		gridX = gridX * self.worldWidth/self.gridWidth
		gridY = gridY * self.worldHeight/self.gridHeight 
		orig_poses = np.matrix([[gridX], [gridY], [0], [-1]])
		final_poses = self.RTGridWorld() * orig_poses
		worldX = final_poses.flat[0]
		worldY = final_poses.flat[1]
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
		
		pose = self.pose3d.getPose3d()
		x = pose.x
		y = pose.y
		#print("x : {} , y : {}".format(x,y))
		scale_y = 0.6  ; offset_y = 153
		y = scale_y * y + offset_y
		
		scale_x = -0.3 ; offset_x = 77
		x = scale_x * x + offset_x
		if x > 77:
			t = x - 77
			x = 77 - t
		else:
			t = 77 - x
			x = t + 77
		#print("x : {} , y : {}".format(x,y))
		return y, x


	def getTaxiAngle(self):
		pose = self.pose3d.getPose3d()
		rt = pose.yaw 
		#print(rt)
		ty = math.cos(-rt) - math.sin(-rt)
		tx = math.sin(-rt) + math.cos(-rt)
		#print("tx : {} , ty : {}".format(tx,ty))
		return tx, ty

	# Function to reset
	def reset(self):
		# Nothing to do, service takes care!
		pass

	def robotPose(self):
		pass

	def getMap(self):
		pass