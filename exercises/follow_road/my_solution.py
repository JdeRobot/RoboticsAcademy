#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from drone_wrapper import DroneWrapper
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Pose

code_live_flag = False

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

def set_image_filtered(img):
	gui_filtered_img_pub.publish(HAL.bridge.cv2_to_imgmsg(img))

def set_image_threshed(img):
	gui_threshed_img_pub.publish(HAL.bridge.cv2_to_imgmsg(img))

def execute(event):
	global HAL
	img_frontal = HAL.get_frontal_image()
	img_ventral = HAL.get_ventral_image()
	# Both the above images are cv2 images
	################# Insert your code here #################################

	set_image_filtered(img_frontal)
	set_image_threshed(img_ventral)

	#########################################################################

if __name__ == "__main__":
	HAL = DroneWrapper()
	rospy.Subscriber('gui/play_stop', Bool, gui_play_stop_cb)
	gui_filtered_img_pub = rospy.Publisher('interface/filtered_img', Image, queue_size = 1)
	gui_threshed_img_pub = rospy.Publisher('interface/threshed_img', Image, queue_size = 1)
	code_live_flag = False
	code_live_timer = rospy.Timer(rospy.Duration(nsecs=50000000), execute)
	code_live_timer.shutdown()
	while not rospy.is_shutdown():
		rospy.spin()
