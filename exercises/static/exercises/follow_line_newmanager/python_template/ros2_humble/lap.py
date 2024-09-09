from datetime import datetime

class Lap:
	def __init__(self, pose3d):
		self.pose3d = pose3d
		self.reset()
		
	# Function to check for threshold
	# And incrementing the running time
	def check_threshold(self):
		pose3d = self.pose3d.getPose3d()
		
		# Variable for pause
		if(self.pause_condition == False):
			# Running condition to calculate the current time
			# Time calculated by adding increments from each iteration
			if(self.start_time != 0 and self.lap_rest == False):
				if(self.lap_time == 0):
					self.lap_time = datetime.now() - self.start_time
				else:
					self.lap_time += datetime.now() - self.start_time

				self.start_time = datetime.now()

			# Condition when the time starts running
			if(self.start_time == 0 and self.lap_rest == True):
				self.start_time = datetime.now()
				self.lap_rest = False

		return self.lap_time
					
	# Function to return lap time
	def return_lap_time(self):
		return self.lap_time

	# Function to reset
	def reset(self):
		# Reset the initial variables
		self.start_time = 0
		self.lap_time = 0

		self.lap_rest = True
		self.buffer = False

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
		   
		   
		   
		   
		   
		   
			
		
