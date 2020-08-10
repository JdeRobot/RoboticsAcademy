import rospy
from datetime import datetime
from interfaces.pose3d import ListenerPose3d

class Lap:
	def __init__(self, topic):
		self.pose3d = ListenerPose3d(topic)
		
		self.threshold_x = [50, 56]
		self.threshold_y = [-13, -12]
		self.threshold_z = [-0.006, 0.006]
		self.threshold_roll = [-0.001, 0.001]
		self.threshold_pitch = [-0.001, 0.001]
		self.threshold_yaw = [-1.9, -1.4]
		
		self.start_time = datetime.now()
		self.lap_time = 0
		# Lap rest is used to register the lap time only once
		self.lap_rest = False
		
	# Function to check for threshold
	def check_threshold(self):
		pose3d = self.pose3d.getPose3d()
		
		threshold_crossed = pose3d.x > self.threshold_x[0] and pose3d.x < self.threshold_x[1] and\
							pose3d.y > self.threshold_y[0] and pose3d.y < self.threshold_y[1] and\
							pose3d.z > self.threshold_z[0] and pose3d.z < self.threshold_z[1]
		
		
		if(threshold_crossed == True and self.lap_rest == False):
			finish_time = datetime.now()
			self.lap_time = finish_time - self.start_time
			self.start_time = finish_time
			self.lap_rest = True
			return self.lap_time
		elif(threshold_crossed == False):
			self.lap_time = 0
			self.lap_rest = False
			
		return None
			
			
	# Function to return lap time
	def return_lap_time(self):
		return self.lap_time
		   
		   
		   
		   
		   
		   
			
		
