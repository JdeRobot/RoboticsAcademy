import rospy
from datetime import datetime

class Lap:
	def __init__(self, map_object):
		self.map = map_object
		
		self.target_start = "target01"
		self.target_end = "NaN"
		
		self.start_time = 0
		self.lap_time = 0
		
		self.buffer = True
		
	# Function to check for threshold
	def check_threshold(self):
		target = self.map.getNextTarget()
		targetid = target.getId()
		
		if(targetid != self.target_end and targetid != self.target_start):
			if(self.buffer == True):
				self.start_time = datetime.now()
				self.buffer = False
				
			self.lap_time = datetime.now() - self.start_time
	
		return self.lap_time
			
			
	# Function to return lap time
	def return_lap_time(self):
		return self.lap_time
		   
		   
		   
		   
		   
		   
			
		
