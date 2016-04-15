import threading
import numpy as np

class Grid:
	def __init__(self, gridW, gridH, worldW, worldH):
		self.lock = threading.Lock()

		self.pos = (gridW/2, gridH/2)
		self.angle = 0
		self.initAngle = 0
		self.destiny = None
		self.path = None
		self.map = None

		self.gWidth = float(gridW)
		self.gHeight = float(gridH)
		self.wWidth = float(worldW)
		self.wHeight = float(worldH)

		self.grid = np.empty([self.gWidth, self.gHeight], float)


	def initPose(self, x, y, angle):
		(gridX, gridY) = self.worldToGrid(x, y)
		self.lock.acquire()
		self.pos = (gridX, gridY)
		self.initAngle = angle * 100
		self.angle = angle * 100
		self.lock.release()

	def worldToGrid(self, worldX, worldY):
		gridX = int((self.gWidth / self.wWidth) * (worldY + self.wWidth/2))
		gridY = int((self.gHeight / self.wHeight) * (worldX + self.wHeight/2))
		return (gridX, gridY)

	def gridToWorld(self, gridX, gridY):
		worldX = (self.wWidth / self.gWidth) * gridY - self.wWidth
		worldY = (self.wHeight / self.gHeight) * gridX - self.wHeight
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

	def setPath(self, path):
		self.path = path

	def getPath(self):
		return self.path

	def setMap(self, mapImage):
		self.map = mapImage

	def getMap(self):
		return self.map