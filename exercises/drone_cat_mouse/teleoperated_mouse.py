#!/usr/bin/env python

import rospy
from drone_wrapper import DroneWrapper
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

def gui_takeoff_cb(msg):
	if msg.data:
		drone.takeoff()
	else:
		drone.land()
		
def gui_twist_cb(msg):
	global drone
	drone.set_cmd_vel(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)

if __name__ == "__main__":
	drone = DroneWrapper()
	rospy.Subscriber('gui/takeoff_land', Bool, gui_takeoff_cb)
	rospy.Subscriber('gui/twist', Twist, gui_twist_cb)
	while not rospy.is_shutdown():
		rospy.spin()