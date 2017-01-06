class Transition:

	def __init__(self, id, init, end, name):
		self.id = id
		self.init = init
		self.end = end
		self.name = name
		self.active = False
		self.orig = [0,0]
		self.dest = [0,0]
		self.angle = 0
		self.pointer = [[0.0],[0,0],[0,0]]

	def setName(self, name):
		self.name = name

	def getName(self):
		return self.name

	def getId(self):
		return self.id

	def getStateInit(self):
		return self.init

	def getStateEnd(self):
		return self.end

	def setOrig(self, x, y):
		self.orig[0] = x
		self.orig[1] = y

	def setDest(self, x, y):
		self.dest[0] = x
		self.dest[1] = y

	def setPointer(self, p):
		self.pointer = p

	def setAngle(self, angle):
		self.angle = angle

	def getOrig(self):
		return self.orig

	def getDest(self):
		return self.dest

	def getPointer(self):
		return self.pointer

	def getAngle(self):
		return self.angle

	def setActive(self, flag):
		self.active = flag

	def isActive(self):
		return self.active

	#def __eq__(self, other):
	#	return self.id == other.id
