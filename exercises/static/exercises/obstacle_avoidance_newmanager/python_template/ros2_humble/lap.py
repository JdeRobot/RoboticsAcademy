from datetime import datetime

class Lap:
	def __init__(self, map_object):
		self.map = map_object
		
		self.target_start = "target01"
		self.target_end = "NaN"
		
		self.reset()
		
	# Function to check for threshold
	def check_threshold(self):
		target = self.map.getNextTarget()
		targetid = target.getId()
		
		# Time activated after collecting the first target
		if(targetid != self.target_end and targetid != self.target_start):
			if(self.buffer == True):
				self.start_time = datetime.now()
				self.buffer = False

			# Time calculated by adding increments from each iteration
			if(self.pause_condition == False):
				if(self.lap_time == 0):	
					self.lap_time = datetime.now() - self.start_time
				else:
					self.lap_time += datetime.now() - self.start_time
				
				self.start_time = datetime.now()
	
		return self.lap_time
			
			
	# Function to return lap time
	def return_lap_time(self):
		return self.lap_time

	# Function to reset
	def reset(self):
		# Reset the initial variables
		self.start_time = 0
		self.lap_time = 0

		self.buffer = True
		self.pause_condition = False

	# Function to pause
	def pause(self):
		self.pause_condition = True

	# Function to unpause
	def unpause(self):
		# To enable unpause button to be used again and again
		if(self.pause_condition == True):
			# Next time the time will be incremented accordingly
			self.start_time = datetime.now()

		self.pause_condition = False
		   
		   
		   
		   
		   
		   
			
		
