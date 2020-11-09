import rospy
from datetime import datetime

class Lap:
	def __init__(self, pose3d):
		self.pose3d = pose3d
		
		self.threshold_x = [50, 56]
		self.threshold_y = [-13, -12]
		self.threshold_z = [-0.006, 0.006]
		self.threshold_roll = [-0.001, 0.001]
		self.threshold_pitch = [-0.001, 0.001]
		self.threshold_yaw = [-1.9, -1.4]
		
		self.reset()
		
	# Function to check for threshold
	def check_threshold(self):
		pose3d = self.pose3d.getPose3d()
		
		threshold_crossed = pose3d.x > self.threshold_x[0] and pose3d.x < self.threshold_x[1] and\
							pose3d.y > self.threshold_y[0] and pose3d.y < self.threshold_y[1] and\
							pose3d.z > self.threshold_z[0] and pose3d.z < self.threshold_z[1]
		
		# Variable for pause
		if(self.pause_condition == False):
			# Running condition to calculate the current time
			if(self.start_time != 0 and self.lap_rest == False):
				self.lap_time = datetime.now() - self.start_time
			
			# Final condition after the lap is complete
			if(threshold_crossed == True and self.lap_rest == False and self.buffer == False):
				finish_time = datetime.now()
				self.lap_time = finish_time - self.start_time
				self.start_time = finish_time
				self.lap_rest = True
				self.buffer = False
				return None

			# Condition when the time starts running
			if(self.start_time == 0 and threshold_crossed == True and self.lap_rest == True):
				self.start_time = datetime.now()
				self.lap_rest = False
				self.buffer = True
				
			# Condition when the the time has started and car is out of threshold
			# This is to allow a buffer for the race car, otherwise the start
			# and finish time are calculated to be the same
			if(threshold_crossed == False and self.buffer == True):
				self.buffer = False

			
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
		self.pause_time = 0

	# Function to pause
	def pause(self):
		self.pause_condition = True
		self.pause_time = self.lap_time

	# Function to unpause
	def unpause(self):
		# To enable unpause button to be used again and again
		if(self.pause_condition == True):
			# Overall subtract from lap_time, the time we have been paused
			self.lap_time = self.lap_time - (datetime.now() - self.pause_time)

		self.pause_condition = False
		   
		   
		   
		   
		   
		   
			
		
