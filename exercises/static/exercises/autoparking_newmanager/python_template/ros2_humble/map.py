import json
import math
import numpy as np

class Map:
	def __init__(self, laser_object_f, laser_object_r, laser_object_b):
		# Define the object used for
		# websocket communication
		self.payload = {}

		self.laser_topic_f = laser_object_f
		self.laser_topic_r = laser_object_r
		self.laser_topic_b = laser_object_b
        
    # Get the JSON data as string
	def get_json_data(self):
		self.payload["lasers"], self.payload["ranges"] = self.setLaserValues()

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
        
	# Interpret the Laser values
	def setLaserValues(self):
		# Init laser array
		lasers = []
		laser_f = []
		laser_r = []
		laser_b = []
		self.laser_f = self.laser_topic_f()
		self.laser_r = self.laser_topic_r()
		self.laser_b = self.laser_topic_b()

		lasers = [self.setSingleLaserValue(self.laser_f), self.setSingleLaserValue(self.laser_r), self.setSingleLaserValue(self.laser_b)]
		ranges = [20 * self.laser_f.maxRange, 20 * self.laser_r.maxRange, 20 * self.laser_b.maxRange]
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