import json
import math
import os
import numpy as np
from interfaces.laser import ListenerLaser

class Map:
	def __init__(self, laser_object_f,pose3d_object):
		# Car direction
		self.carx = 2.0
		self.cary = 0.0

		# Obstacles direction
		self.obsx = 0.0
		self.obsy = 2.0

		# Average direction
		self.avgx = -2.0
		self.avgy = 0.0

		# Define the object used for
		# websocket communication
		self.payload = {}

		self.laser_topic_f = laser_object_f
		self.pose3d = pose3d_object
        
    # Get the JSON data as string
	def get_json_data(self):								
		self.payload["car"] = self.setArrow(self.carx, self.cary)
		self.payload["obstacle"] = self.setArrow(self.obsx, self.obsy)
		self.payload["average"] = self.setArrow(self.avgx, self.avgy)
		self.payload["lasers"], self.payload["ranges"] = self.setLaserValues()
		#self.payload["max_range"] = self.laser.maxRange

		message = json.dumps(self.payload)
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

        
    # Interpret the Target values
	def setTarget(self, x, y, rx, ry, rt):
		# Convert to relatives
		if x == 0.0 and y == 0.0:
			return (0, 0)
			
		dx = rx - x
		dy = ry - y

		# Rotate with the current angle
		ty = dx*math.cos(-rt) - dy*math.sin(-rt)
		tx = dx*math.sin(-rt) + dy*math.cos(-rt)
		
		ty = (120 +  20 * ty)
		tx = (146.5 + 7 * tx)

		return (tx, ty)
    
    # Interpret the arrow values	
	def setArrow(self, posx, posy):
		return (posx, posy)
        
    # Interpret the Laser values
	def setLaserValues(self):
		# Init laser array
		lasers = []
		laser_f = []
		laser_r = []
		laser_b = []
		self.laser_f = self.laser_topic_f.getLaserData()

		lasers = [self.setSingleLaserValue(self.laser_f)]
		ranges = [20 * self.laser_f.maxRange]
		return lasers, ranges

	# Auxiliar function to setLaserValues
	def setSingleLaserValue(self, laser):
		laser_return = []
		if(laser):
			angle = int(round(math.degrees(laser.maxAngle)))
		else:
			angle = 180

		for i in range(angle):
			laser_return.append((0, 0))
				
		if(laser):
			for i in range(angle):
				dist = 20 * laser.values[i]
				angle = math.radians(i)
				if(dist == float("inf")):
					dist = 20 * laser.maxRange
				laser_return[i] = (dist, angle)

		return laser_return
