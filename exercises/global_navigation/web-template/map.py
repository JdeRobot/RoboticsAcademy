import numpy as np
import math
from math import pi as pi
import cv2

class Map:
	def __init__(self, pose3d):
		self.pose3d = pose3d
	
	def RTx(self, angle, tx, ty, tz):
		RT = np.matrix([[1, 0, 0, tx], [0, math.cos(angle), -math.sin(angle), ty], 
						[0, math.sin(angle), math.cos(angle), tz], [0, 0, 0, 1]])
		return RT
		
	def RTy(self, angle, tx, ty, tz):
		RT = np.matrix([[math.cos(angle), 0, math.sin(angle), tx], [0, 1, 0, ty],
						[-math.sin(angle), 0, math.cos(angle), tz], [0, 0, 0, 1]])
		return RT
		
	def RTz(self, angle, tx, ty, tz):
		RT = np.matrix([[math.cos(angle), -math.sin(angle), 0, tx], [math.sin(angle), math.cos(angle), 0, ty],
						[0, 0, 1, tz], [0, 0, 0, 1]])
		return RT
		
	def RTFormula(self):
		RTz = self.RTz(pi/2, 50, 70, 0)
		return RTz
		
	def getTaxiCoordinates(self):
		
		pose = self.pose3d.getPose3d()
                #print(pose)
		x = pose.x 
		y = pose.y 
		#print("x : {} , y : {}".format(x,y))
		scale_y = 0.6  ; offset_y = 153
		y = scale_y * y + offset_y
		
		scale_x = -0.3 ; offset_x = 77
		x = scale_x * x + offset_x
                if x > 77:
                   t = x - 77
                   x = 77 - t
                else:
                   t = 77 - x
                   x = t + 77
                   
		#print("x : {} , y : {}".format(x,y))
		return y, x 
        
        def getTaxiAngle(self):
		pose = self.pose3d.getPose3d()
		rt = pose.yaw 
                #print(rt)
		ty = math.cos(-rt) - math.sin(-rt)
		tx = math.sin(-rt) + math.cos(-rt)
                #print("tx : {} , ty : {}".format(tx,ty))
		return tx, ty
	# Function to reset
	def reset(self):
		# Nothing to do, service takes care!
		pass
	
