import json
import math
import os
import numpy as np
from hal_interfaces.general.odometry import Pose3d

class Map:
	def __init__(self, laser_object, pose_callback):
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
		directory = os.path.dirname(os.path.realpath(__file__))
		parser = Parser(directory + "/targets_f1.json")
		self.targets = parser.getTargets()

		# Define the object used for
		# websocket communication
		self.payload = {}

		self.laser_callback = laser_object
		self.pose_callback = pose_callback
    
	def setCar(self, newx, newy):
		self.carx = newx
		self.cary = newy
	
	def setObs(self, newx, newy):
		self.obsx = newx
		self.obsy = newy
	
	def setAvg(self, newx, newy):
		self.avgx = newx
		self.avgy = newy    

	def setTargetPos(self, newx, newy):
		self.targetx = newx
		self.targety = newy

    # Get the JSON data as string
	def get_json_data(self):
		self.payload["pose"] = self.setPose(self.pose_callback())
		self.payload["target"] = self.setTarget(self.targetx, self.targety)
		self.payload["car"] = self.setArrow(self.carx, self.cary)
		self.payload["obstacle"] = self.setArrow(self.obsx, self.obsy)
		self.payload["average"] = self.setArrow(self.avgx, self.avgy)
		self.payload["laser"], self.payload["max_range"] = self.setLaserValues()
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
        
    # Target function
    # Function to get information of next target
	def getNextTarget(self):
		for target in self.targets:
			if target.isReached() == False:
				self.setTargetPos(target.pose.x, target.pose.y)
				return target
		return self.resetTargets()
	
	# Function to reset all targets once the last one is reached
	def resetTargets(self):
		for target in self.targets:
			target.setReached(False)
		return self.targets[0]

    # Function to reset target information
	def reset(self):
		for target in self.targets:
			target.setReached(False)

    # Interpret the Target values
	def setPose(self, pose):
		return [pose.x, pose.y, pose.yaw]

    # Interpret the Target values
	def setTarget(self, x, y):
		return [x, y]
    
    # Interpret the arrow values	
	def setArrow(self, posx, posy):
		return (posx, posy)
        
    # Interpret the Laser values
	def setLaserValues(self):
		# Init laser array
		laser = []
		self.laser = self.laser_callback()

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
				if(dist == float("inf")):
					dist = 20 * self.laser.maxRange
				laser[i] = (dist, angle)
						
		return laser, 20 * self.laser.maxRange

class Parser:
    def __init__(self,filename):
        with open(filename) as data_file:
            self.data = json.load(data_file)

    def getTargets(self):
        targets = []
        for t in self.data["targets"]:
            pose = Pose3d()
            pose.x = t["x"]; pose.y = t["y"]
            targets.append(Target(t["name"],pose,False,False))
        return targets

class Target:
    def __init__(self,id,pose,active=False,reached=False):
        self.id=id
        self.pose=pose
        self.active=active
        self.reached=reached

    def getPose(self):
        return self.pose

    def getId(self):
        return self.id

    def getPose(self):
        return self.pose

    def isReached(self):
        return self.reached

    def setReached(self,value):
        self.reached=value

    def isActive(self):
        return self.active

    def setActive(self,value):
        self.active=True
