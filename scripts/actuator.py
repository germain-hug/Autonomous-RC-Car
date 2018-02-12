#!/usr/bin/env python

"""
Reads commands through the 'cmd' topic, controls the motors
"""

import os
import usb.core
import usb.util
import rospy
from geometry_msgs.msg import Point

class Actuator(object):

	device = None
	configuration = None
	
	def callback(self, data):
		# Pre-process input
		throttle = max(min(data.x, 0.9), -0.9)
		steering = -max(min(data.y, 0.9), -0.9)
		# Send throttle and steering commands
		self.device.ctrl_transfer(0x40, 0x85, int((1500 + 250 * throttle) * 4), 1)
		self.device.ctrl_transfer(0x40, 0x85, int((1500 + 500 * steering) * 4), 0)
		print(throttle, steering)

	def __init__(self):
		# Connect to Pololu through USB
		self.device = usb.core.find(idVendor=0x1ffb, idProduct=0x89)
		self.configuration = self.device.get_active_configuration()

		# Initialize ROS Subscriber
		if self.device is not None:
			rospy.init_node('actuator', anonymous=True)
			rospy.Subscriber('cmd', Point, self.callback)
			rospy.spin()
		else:
			rospy.logerr("Pololu device not detected!")

if __name__ == '__main__':
    actuator = Actuator()
    actuator.init()
