from Parser import Parser
import json
import math
import numpy as np
from interfaces.laser import ListenerLaser

class Map:
	def __init__(self, laser_object, pose3d_object):
		# Car direction
		self.carx = 2.0
		self.cary = 0.0

		# Obstacles direction
		self.obsx = 0.0
		self.obsy = 2.0

		# Average direction
		self.avgx = -2.0
		self.avgy = 0.0

		# Current target
		self.targetx = 0.0
		self.targety = 0.0
		self.targetid = "Nan"

		# Init targets
		parser = Parser("targets_f1.json")
		self.targets = parser.getTargets()

		# Define the object used for
		# websocket communication
		self.payload = {}

		self.laser_topic = laser_object
		self.pose3d = pose3d_object
        
    # Get the JSON data as string
	def get_json_data(self):
		self.payload["target"] = self.setTarget(self.targetx, self.targety,
								self.pose3d.getPose3d().x, 
								self.pose3d.getPose3d().y,
								self.pose3d.getPose3d().yaw)
								
		self.payload["car"] = self.setArrow(self.carx, self.cary)
		self.payload["obstacle"] = self.setArrow(self.obsx, self.obsy)
		self.payload["average"] = self.setArrow(self.avgx, self.avgy)
		self.payload["laser"], self.payload["max_range"] = self.setLaserValues()
		#self.payload["max_range"] = self.laser.maxRange

		message = "#map" + json.dumps(self.payload)
		return message
    	
	def RTx(self, angle, tx, ty, tz):
		RT = np.matrix([[1, 0, 0, tx], [0, math.cos(angle), -math.sin(angle), ty], [0, math.sin(angle), math.cos(angle), tz], [0,0,0,1]])
		return RT
        
	def RTy(self, angle, tx, ty, tz):
		RT = np.matrix([[math.cos(angle), 0, math.sin(angle), tx], [0, 1, 0, ty], [-math.sin(angle), 0, math.cos(angle), tz], [0,0,0,1]])
		return RT
    
	def RTz(self, angle, tx, ty, tz):
		RT = np.matrix([[math.cos(angle), -math.sin(angle), 0, tx], [math.sin(angle), math.cos(angle),0, ty], [0, 0, 1, tz], [0,0,0,1]])
		return RT  	
        
    # Target function
    # Function to get information of next target
	def getNextTarget(self):
		for target in self.targets:
			if target.isReached() == False:
				return target

		return None
        
    # Interpret the Target values
	def setTarget(self, x, y, rx, ry, rt):
		# Convert to relatives
		if x == 0.0 and y == 0.0:
			pass
			
		dx = rx - x
		dy = ry - y

		# Rotate with the current angle
		tx = dx*math.cos(-rt) - dy*math.sin(-rt)
		ty = dx*math.sin(-rt) + dy*math.cos(-rt)

		return (tx, ty)
    
    # Interpret the arrow values	
	def setArrow(self, posx, posy):
		return (posx, posy)
        
    # Interpret the Laser values
	def setLaserValues(self):
		# Init laser array
		laser = []
		self.laser = self.laser_topic.getLaserData()

		if(self.laser):
			angle = int(round(math.degrees(self.laser.maxAngle)))
		else:
			angle = 180

		for i in range(angle):
			laser.append((0, 0))
				
		if(self.laser):
			for i in range(angle):
				dist = 20 * self.laser.values[i]
				angle = math.radians(i)
				laser[i] = (dist, angle)
						
		return laser, 20 * self.laser.maxRange
