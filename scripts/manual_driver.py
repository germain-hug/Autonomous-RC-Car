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
from datetime import timedelta, date
from std_msgs.msg import String
from geometry_msgs.msg import Point

import cv2
import time
import imutils
from imutils.video import WebcamVideoStream


class ManualDriver(object):

	# Bluetooth controller
	controller = None
	axis_data = None
	button_data = None
	hat_data = None

	curr_mode = 'manual'
	recording = False 
	recording_folder = None
	recording_csv = None
	pub = None

	img_count = 0
	videoStream = WebcamVideoStream()

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

		# Open Camera stream
		self.videoStream.start()
		time.sleep(1.0)

	def __exit__(self, exc_type, exc_value, traceback):
		self.videoStream.stop()


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
						if(event.button==0):
							rospy.loginfo("Starting recording...")
							self.recording = True
							self.recording_folder = '../data/' + str(date.today()) + "-" + str(time.strftime("%H-%M-%S")) + "/"
							if not os.path.exists(self.recording_folder):
								os.makedirs(self.recording_folder)
							self.recording_csv.open(self.recording_folder + 'annotations.csv', 'a')

						# Stopping Recording
						elif(event.button==2):
							rospy.loginfo("Stopping recording...")
							self.recording_csv.close()
							self.recording = False

					# Acceleration and Steering
					elif event.type == pygame.JOYAXISMOTION:

						self.axis_data[event.axis] = round(event.value,2)
						throttle = round(0.5 + self.axis_data[4] / 2.0, 5)
						steering = round(self.axis_data[0], 5)
						self.pub.publish(Point(throttle,steering,0.0))

						if(self.recording):
							frame = self.videoStream.read()
							img_path = recording_folder + 'img_' + img_count + '.jpg'
							cv2.imwrite(img_path, frame)
							self.recording_csv.write([img_count + '.jpg', throttle, steering])
							self.img_count += 1





if __name__ == "__main__":
	manual_driver = ManualDriver()
	manual_driver.listen()
