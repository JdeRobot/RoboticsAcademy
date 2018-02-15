import threading
import math
import numpy as np
import cv2
from math import pi as pi

class Grid:
	def __init__(self, frame):
		self.lock = threading.Lock()

		self.destiny = None
		self.pathFinded = False
		self.map = None

		self.gWidth = int(frame.width())
		self.gHeight = int(frame.height())
		self.wWidth = float(frame.worldWidth())
		self.wHeight = float(frame.worldHeight())
		self.origX = frame.origX()
		self.origY = frame.origY()
		self.mapAngle = frame.mapAngle()

		self.grid = np.empty([self.gWidth, self.gHeight], float)
		self.path = np.zeros([self.gWidth, self.gHeight])


	def initPose(self, x, y, angle):
		print("RX:", x, "RY:", y)
		(gridX, gridY) = self.worldToGrid(x, y)
		print("GX:", gridX, "GY:", gridY)
		self.lock.acquire()
		self.pos = (gridX, gridY)
		self.initAngle = angle * 180 / math.pi
		self.angle = angle * 180 / math.pi
		self.lock.release()


	def RTx(self, angle, tx, ty, tz):
		RT = np.matrix([[1, 0, 0, tx], [0, math.cos(angle), -math.sin(angle), ty], [0, math.sin(angle), math.cos(angle), tz], [0,0,0,1]])
		return RT
        
	def RTy(self, angle, tx, ty, tz):
		RT = np.matrix([[math.cos(angle), 0, math.sin(angle), tx], [0, 1, 0, ty], [-math.sin(angle), 0, math.cos(angle), tz], [0,0,0,1]])
		return RT
    
	def RTz(self, angle, tx, ty, tz):
		RT = np.matrix([[math.cos(angle), -math.sin(angle), 0, tx], [math.sin(angle), math.cos(angle),0, ty], [0, 0, 1, tz], [0,0,0,1]])
		return RT

	def RTWorldGrid(self):
		RTx = self.RTx(pi, 0, 0, 0)
		RTz = self.RTz(-pi/2, self.gWidth/2, -self.gHeight/2, 0)
		return RTx*RTz

	def RTGridWorld(self):
		RTx = self.RTx(-pi, 0, 0, 0)
		RTz = self.RTz(pi/2, self.wWidth/2, self.wHeight/2, 0)
		return RTz*RTx

	def worldToGrid(self, worldX, worldY):
		# self.gWidth/self.wWidth is the scale
		worldX = worldX * self.gWidth/self.wWidth
		worldY = worldY * self.gHeight/self.wHeight
		orig_poses = np.matrix([[worldX], [worldY], [0], [1]])
		final_poses = self.RTWorldGrid() * orig_poses
        
		gridX = int(final_poses.flat[0])
		gridY = int(final_poses.flat[1])
		return (gridX, gridY)


	def gridToWorld(self, gridX, gridY):
		# self.wWidth/self.gWidth is the scale
		gridX = gridX * self.wWidth/self.gWidth
		gridY = gridY * self.wHeight/self.gHeight
		orig_poses = np.matrix([[gridX], [gridY], [0], [-1]])
		final_poses = self.RTGridWorld() * orig_poses
        
		worldX = final_poses.flat[0]
		worldY = final_poses.flat[1]
		return (worldX, worldY)


	def updatePose(self, px, py, angle):
		(gridX, gridY) = self.worldToGrid(px, py)	
		self.lock.acquire()
		self.pos = (gridX, gridY)
		# Convert to degrees
		self.angle = angle * 180 / math.pi
		self.lock.release()

	def getAngle(self):
		self.lock.acquire()
		tmp = self.angle - self.initAngle
		self.lock.release()
		return tmp

	def getPose(self):
		self.lock.acquire()
		tmp = self.pos
		self.lock.release()
		return tmp

	def setDestiny(self, x, y):
		self.lock.acquire()
		self.destiny = (x, y)
		f = open("sensors/destiny.txt", "w")
		f.write(str(self.destiny))
		f.close()
		self.lock.release()

	def getDestiny(self):
		self.lock.acquire()
		tmp = self.destiny
		self.lock.release()
		return tmp

	def setPathVal(self, x, y, val):
		self.lock.acquire()
		self.path[y][x] = val
		self.lock.release()

	def getPathVal(self, y, x):
		self.lock.acquire()
		tmp = self.path[y][x]
		self.lock.release()
		return tmp

	def getPath(self):
		self.lock.acquire()
		tmp = self.path
		self.lock.release()
		return tmp

	def setPathFinded(self):
		self.pathFinded = True

	def setMap(self, mapImage):
		self.map = mapImage

	def getMap(self):
		return self.map

	def getVal(self, x, y):
		self.lock.acquire()
		tmp = self.grid[y][x]
		self.lock.release()
		return tmp

	def setVal(self, x, y, val):
		self.lock.acquire()
		self.grid[y][x] = val
		self.lock.release()

	def resetPath(self):
		self.lock.acquire()
		self.path = np.zeros([self.gWidth, self.gHeight])
		self.pathFinded = False
		self.lock.release()

	def resetGrid(self):
		self.lock.acquire()
		self.grid = np.empty([self.gWidth, self.gHeight], float)
		self.lock.release()

	def getWidth(self):
		return self.gWidth

	def getHeight(self):
		return self.gHeight

	def showGrid(self):
		self.lock.acquire()
		maxVal = np.amax(self.grid)
		if maxVal != 0:
			nCopy = np.dot(self.grid, (1/maxVal))
		else:
			 nCopy = self.grid
		cv2.imshow("Grid Field", nCopy)
		self.lock.release()
