#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import math
from math import pi
from drone_wrapper import DroneWrapper
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

code_live_flag = False

def gui_takeoff_cb(msg):
	if msg.data:
		drone.takeoff()
	else:
		drone.land()

def gui_play_stop_cb(msg):
	global code_live_flag, code_live_timer
	if msg.data == True:
		if not code_live_flag:
			code_live_flag = True
			code_live_timer = rospy.Timer(rospy.Duration(nsecs=50000000), execute)
	else:
		if code_live_flag:
			code_live_flag = False
			code_live_timer.shutdown()
		
def gui_twist_cb(msg):
	global drone
	drone.set_cmd_vel(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)

def set_image_filtered(img):
	gui_filtered_img_pub.publish(drone.bridge.cv2_to_imgmsg(img))

def set_image_threshed(img):
	gui_threshed_img_pub.publish(drone.bridge.cv2_to_imgmsg(img))

################# Build your position control function here #####################

def position_control():
	global drone
	# Insert your code here
 
#################################################################################

def execute(event):
	global drone

	################# Insert your code here #################################

	# Waypoint list  
	waypoint_list = []
	
	# Navigation using position control. Same tolerance for all
	for waypoint in waypoint_list:
	    position_control()
			
	#########################################################################

if __name__ == "__main__":
	drone = DroneWrapper()
	rospy.Subscriber('gui/takeoff_land', Bool, gui_takeoff_cb)
	rospy.Subscriber('gui/play_stop', Bool, gui_play_stop_cb)
	rospy.Subscriber('gui/twist', Twist, gui_twist_cb)
	gui_filtered_img_pub = rospy.Publisher('interface/filtered_img', Image, queue_size = 1)
	gui_threshed_img_pub = rospy.Publisher('interface/threshed_img', Image, queue_size = 1)
	code_live_flag = False
	code_live_timer = rospy.Timer(rospy.Duration(nsecs=50000000), execute)
	code_live_timer.shutdown()
	while not rospy.is_shutdown():
		rospy.spin()
