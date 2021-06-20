#!/usr/bin/env python

import rospy
from drone_wrapper import DroneWrapper
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from math import sin as s
from math import cos as c
from random import uniform as u
from time import time as t

code_live_flag = False
a = 0

##############################
# FOUR PATH LEVELS AVAILABLE #
##############################
PATH = 0  # 0, 1, 2, 3


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


def path_obfuscated(b):
	global a;r=round;d=(lambda x,y:(lambda x:((lambda x:3 if r(x/2)%2==0 else 0)(x),(lambda x:0)(x),(lambda x:0)(x),(lambda x:0)(x)))(y)if x==0 else((lambda x:((lambda x:1)(x),(lambda x:-2 if r(x/4)%2==0 else 2)(x),(lambda x:0)(x),(lambda x:0)(x)))(y)if x==1 else((lambda x:((lambda x:1)(x),(lambda x:s(x/2)*1)(x),(lambda x:c(x/2)*1)(x),(lambda x:0)(x)))(y)if x==2 else((lambda x:((lambda x:1)(x),(lambda x:0)(x),(lambda x:0)(x),(lambda x:u(-1,1) if r(x/2%2,2)<=0.05 else a)(x)))(y)if x==3 else None))))(b,t());a=(d[3]if d is not None else 0);return d


def execute(event):
	global drone

	v = path_obfuscated(PATH)
	if v is not None:
		vx, vy, vz, yaw = v
		drone.set_cmd_vel(vx, vy, vz, yaw)
	else:
		print("[Mouse] Path {} not available".format(PATH))


def gui_twist_cb(msg):
	global drone
	drone.set_cmd_vel(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)


if __name__ == "__main__":
	drone = DroneWrapper()
	rospy.Subscriber('gui/takeoff_land', Bool, gui_takeoff_cb)
	rospy.Subscriber('gui/play_stop', Bool, gui_play_stop_cb)
	rospy.Subscriber('gui/twist', Twist, gui_twist_cb)

	code_live_flag = False
	code_live_timer = rospy.Timer(rospy.Duration(nsecs=50000000), execute)
	code_live_timer.shutdown()
	while not rospy.is_shutdown():
		rospy.spin()