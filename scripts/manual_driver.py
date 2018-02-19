#! /usr/bin/env python

"""
- Activated when driving in manual mode
- Press 'O' to record
- Press 'square' to stop recording
- Use Left / Right joystick to steer
- Use L2 to control speed

"""

import os
import time
import pygame
import rospy
import imutils

from datetime import timedelta, date
from std_msgs.msg import String
from geometry_msgs.msg import Point
from imutils.video import WebcamVideoStream
from save_data import DataSaver

class ManualDriver(object):

	# Bluetooth controller
	controller = None
	axis_data = None
	button_data = None
	hat_data = None

	curr_mode = 'manual'
	recording = False
	pub = None
	def callback(self, data):
		self.curr_mode = data.data
		rospy.loginfo("Switched to mode: " + self.curr_mode)

	def __init__(self):   
		# Initialize joystick receiver using pygame
		pygame.init()
		pygame.joystick.init()
		while self.controller is None:
			try:
				self.controller = pygame.joystick.Joystick(0)
			except:
				pass
		self.controller.init()

		# Initialize ROS Subscriber
		rospy.init_node('manual_driver', anonymous=True)
		rospy.Subscriber('mode', String, self.callback)
		self.pub = rospy.Publisher('cmd', Point, queue_size=1)

	# Listen for Controller inputs
	def listen(self):

		# Initialize commands
		if not self.axis_data:
			self.axis_data = {0:0.0,1:0.0,2:0.0,3:0.0,4:-1.0}
		if not self.button_data:
			self.button_data = {}
			for i in range(self.controller.get_numbuttons()):
				self.button_data[i] = False
		if not self.hat_data:
			self.hat_data = {}
			for i in range(self.controller.get_numhats()):
				self.hat_data[i] = (0, 0)

		# Main listening loop
		while not rospy.is_shutdown():
			if self.curr_mode =='manual':
				for event in pygame.event.get():

					# Stop and start recording
					if event.type == pygame.JOYBUTTONDOWN:

						# Starting Recording
						if(event.button==2 and not self.recording):
							rospy.loginfo("Starting recording...")
							self.recording = True
							self.data_saver = DataSaver('/home/nvidia/jetsonbot/src/rccar/data/' + str(date.today()) + "-" + str(time.strftime("%H-%M-%S")) + "/")
						# Stopping Recording
						elif(event.button==0 and self.recording):
							rospy.loginfo("Stopping recording...")
							del self.data_saver
		
					# Acceleration and Steering
					elif event.type == pygame.JOYAXISMOTION:
						self.axis_data[event.axis] = round(event.value,2)
						throttle = round(0.5 + self.axis_data[4] / 2.0, 5)
						steering = round(self.axis_data[0], 5)
						self.pub.publish(Point(throttle,steering,0.0))
						


if __name__ == "__main__":
	manual_driver = ManualDriver()
	manual_driver.listen()
