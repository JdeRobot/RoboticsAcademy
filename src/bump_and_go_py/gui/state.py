from gui.transition import *

class State:

	def __init__(self, id, name):
		self.id = id
		self.name = name
		self.transitions = []
		self.active = False
		self.x = 0
		self.y = 0


	def addTransition(self, id, end, name):
		t = Transition(id, self.id, end, name)
		self.transitions.append(t)
		return t

	def getTransitions(self):
		return self.transitions

	def getTransition(self, n):
		return self.transitions[n]

	def setName(self, name):
		self.name = name

	def getName(self):
		return self.name

	def getId(self):
		return self.id	

	def setActive(self, flag):
		self.active = flag

	def isActive(self):
		return self.active

	def setPos(self, x, y):
		self.x = x
		self.y = y

	def getPos(self):
		return [self.x, self.y]
