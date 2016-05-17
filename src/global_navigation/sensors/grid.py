import threading
import math
import numpy as np
import cv2

class Grid:
	def __init__(self, frame):
		self.lock = threading.Lock()

		self.destiny = None
		self.pathFinded = False
		self.map = None

		self.gWidth = float(frame.width())
		self.gHeight = float(frame.height())
		self.wWidth = float(frame.worldWidth())
		self.wHeight = float(frame.worldHeight())
		self.origX = frame.origX()
		self.origY = frame.origY()
		self.mapAngle = frame.mapAngle()

		self.grid = np.empty([self.gWidth, self.gHeight], float)
		self.path = np.zeros([self.gWidth, self.gHeight])


	def initPose(self, x, y, angle):
		(gridX, gridY) = self.worldToGrid(x, y)
		self.lock.acquire()
		self.pos = (gridX, gridY)
		self.initAngle = angle * 100
		self.angle = angle * 100
		self.lock.release()


	def worldToGrid(self, worldX, worldY):
		if self.mapAngle >= 0 and self.mapAngle < 90:
			relativeAngle = self.mapAngle * math.pi / 180
			gridX = int((self.gWidth * math.cos(relativeAngle) / self.wWidth) * (worldX - self.origX))
			gridY = int((-self.gHeight * math.cos(relativeAngle) / self.wHeight) * (worldY - self.origY))

		elif self.mapAngle >= 90 and self.mapAngle < 180:
			relativeAngle = (self.mapAngle - 90) * math.pi / 180
			gridX = int((self.gHeight * math.cos(relativeAngle) / self.wHeight) * (worldY - self.origX))
			gridY = int((self.gWidth * math.cos(relativeAngle) / self.wWidth) * (worldX - self.origY))

		elif self.mapAngle >= 180 and self.mapAngle < 270:
			relativeAngle = (self.mapAngle - 180) * math.pi / 180
			gridX = int((-self.gWidth * math.cos(relativeAngle) / self.wWidth) * (worldX - self.origX))
			gridY = int((self.gHeight * math.cos(relativeAngle) / self.wHeight) * (worldY - self.origY))

		elif self.mapAngle >= 270 and self.mapAngle < 360:
			relativeAngle = (self.mapAngle - 270) * math.pi / 180
			gridX = int((-self.gHeight * math.cos(relativeAngle) / self.wHeight) * (worldY - self.origX))
			gridY = int((-self.gWidth * math.cos(relativeAngle) / self.wWidth) * (worldX - self.origY))

		return (gridX, gridY)


	def gridToWorld(self, gridX, gridY):
		if self.mapAngle >= 0 and self.mapAngle < 90:
			relativeAngle = self.mapAngle * math.pi / 180
			worldX = (self.wWidth / self.gWidth) * gridX * math.cos(relativeAngle) + self.origX
			worldY = -(self.wHeight / self.gHeight) * gridY * math.cos(relativeAngle) + self.origY

		elif self.mapAngle >= 90 and self.mapAngle < 180:
			relativeAngle = (self.mapAngle - 90) * math.pi / 180
			worldX = (self.wHeight / self.gHeight) * gridY * math.cos(relativeAngle) + self.origX
			worldY = (self.wWidth / self.gWidth) * gridX * math.cos(relativeAngle) + self.origY

		elif self.mapAngle >= 180 and self.mapAngle < 270:
			relativeAngle = (self.mapAngle - 180) * math.pi / 180
			worldX = -(self.wWidth / self.gWidth) * gridX * math.cos(relativeAngle) + self.origX
			worldY = (self.wHeight / self.gHeight) * gridY * math.cos(relativeAngle) + self.origY

		elif self.mapAngle >= 270 < 360:
			relativeAngle = (self.mapAngle - 270) * math.pi / 180
			worldX = -(self.wHeight / self.gHeight) * gridY * math.cos(relativeAngle) + self.origX
			worldY = -(self.wWidth / self.gWidth) * gridX * math.cos(relativeAngle) + self.origY

		return (worldX, worldY)


	def updatePose(self, px, py, angle):
		(gridX, gridY) = self.worldToGrid(px, py)

		self.lock.acquire()
		self.pos = (gridX, gridY)
		self.angle = angle * 100
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