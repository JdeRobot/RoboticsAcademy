import numpy as np
import threading
import time
from datetime import datetime
import jderobot
import math
from Target import Target
from Parser import Parser

time_cycle = 80

class MyAlgorithm(threading.Thread):

	def __init__(self, pose3d, laser, motors):
		self.pose3d = pose3d
		self.laser = laser
		self.motors = motors

		# Car direction
		self.carx = 0.0
		self.cary = 0.0

		# Obstacles direction
		self.obsx = 0.0
		self.obsy = 0.0

		# Average direction
		self.avgx = 0.0
		self.avgy = 0.0

		# Current target
		self.targetx = 0.0
		self.targety = 0.0
		self.targetid = "NaN"

		self.stop_event = threading.Event()
		self.kill_event = threading.Event()
		self.lock = threading.Lock()
		threading.Thread.__init__(self, args=self.kill_event)

		# Init targets
		parser = Parser('targets_f1.json')
		self.targets = parser.getTargets()

	def getNextTarget(self):
		for target in self.targets:
			if target.isReached() == False:
			    return target

		return None

	def getCarDirection(self):
		return (self.carx, self.cary)

	def getObstaclesDirection(self):
		return (self.obsx, self.obsy)

	def getAverageDirection(self):
		return (self.avgx, self.avgy)

	def getCurrentTarget(self):
		return (self.targetx, self.targety, self.targetid)

	def run (self):

		while (not self.kill_event.is_set()):
			           
			start_time = datetime.now()

			if not self.stop_event.is_set():
			    self.execute()

			finish_Time = datetime.now()

			dt = finish_Time - start_time
			ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
			#print (ms)
			if (ms < time_cycle):
			    time.sleep((time_cycle - ms) / 1000.0)


	def stop (self):
		self.stop_event.set()

	def play (self):
		if self.is_alive():
			self.stop_event.clear()
		else:
			self.start()

	def kill (self):
		self.kill_event.set()

	def execute(self):
		print "running"
		self.currentTarget=self.getNextTarget()
		self.targetx = self.currentTarget.getPose().x
		self.targety = self.currentTarget.getPose().y
		self.targetid = self.currentTarget.getId()
		#print(self.targetx,self.targety)

		# TODO

		# Car direction <GREEN>
		self.carx = 0.0
		self.cary = 2.0

		# Obstacles direction <RED>
		self.obsx = 2.0
		self.obsy = 0.0

		# Average direction <BLACK>
		self.avgx = 2.0
		self.avgy = 2.0

