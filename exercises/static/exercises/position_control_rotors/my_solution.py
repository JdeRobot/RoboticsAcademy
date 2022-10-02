#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from drone_wrapper import DroneWrapper
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Pose

from Beacon import Beacon

code_live_flag = False
beacons = []
min_error = 0.01

def init_beacons():
	global beacons
	beacons.append(Beacon('beacon1', np.array([0, 5, 0]), False, False))
	beacons.append(Beacon('beacon2', np.array([5, 0, 0]), False, False))
	beacons.append(Beacon('beacon3', np.array([0, -5, 0]), False, False))
	beacons.append(Beacon('beacon4', np.array([-5, 0, 0]), False, False))
	beacons.append(Beacon('beacon5', np.array([10, 0, 0]), False, False))
	beacons.append(Beacon('initial', np.array([0, 0, 0]), False, False))

def get_next_beacon():
	for beacon in beacons:
		if beacon.is_reached() == False:
			return beacon
	return None

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
	gui_filtered_img_pub.publish(drone.bridge.cv2_to_imgmsg(img))

def set_image_threshed(img):
	gui_threshed_img_pub.publish(drone.bridge.cv2_to_imgmsg(img))

def execute(event):
	global drone
	img_frontal = drone.get_frontal_image()
	img_ventral = drone.get_ventral_image()
	# Both the above images are cv2 images
	################# Insert your code here #################################

	set_image_filtered(img_frontal)
	set_image_threshed(img_ventral)

	#########################################################################

if __name__ == "__main__":
	drone = DroneWrapper()
	rospy.Subscriber('gui/play_stop', Bool, gui_play_stop_cb)
	gui_filtered_img_pub = rospy.Publisher('interface/filtered_img', Image, queue_size = 1)
	gui_threshed_img_pub = rospy.Publisher('interface/threshed_img', Image, queue_size = 1)
	code_live_flag = False
	code_live_timer = rospy.Timer(rospy.Duration(nsecs=50000000), execute)
	code_live_timer.shutdown()
	while not rospy.is_shutdown():
		rospy.spin()
