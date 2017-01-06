from gui.state import *

class Machine:

	def __init__(self, n):

		self.states = []
		self.transitions = []
		self.cont = 0
		self.t_cont = 0
		
		for i in range (n):
			state = State(self.cont, '')
			self.states.append(state)
			self.cont += 1

	def addState(self, name):
		state = State(cont, name)
		self.states.append(state)
		self.cont += 1

	def addTransition(self, orig, end, name):
		t = self.states[orig].addTransition(self.t_cont, end, name)
		self.t_cont += 1
		if not t in self.transitions:
			self.transitions.append(t)

	def getState(self, arg):
		if type(arg) == int:
			return self.states[arg]
		else:
			for s in self.states:
				if s.getName() == arg:
					return s

	def getStates(self):
		return self.states

	def getActiveTransition(self):
		for t in self.transitions:
			if t.isActive():
				return t
				
	def getTransitions(self):
		return self.transitions

	def setStateActive(self, n_state, flag):
		self.deactivateAll()
		self.states[n_state].setActive(flag)

	def setTransitionActive(self, n_transition, flag):
		self.deactivateAll()
		self.transitions[n_transition].setActive(flag)

	def setStateName(self, n_state, name):
		self.states[n_state].setName(name)	

	def isStateActive(self, n_state):
		return self.states[n_state].isActive()

	def deactivateAll(self):
		for state in self.states:
			state.setActive(False)
		for transition in self.transitions:
				transition.setActive(False)
