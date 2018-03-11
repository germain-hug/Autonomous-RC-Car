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
from geometry_msgs.msg import Point

import cv2
import time
import imutils
from imutils.video import WebcamVideoStream

class DataSaver(object):
	img_count = 0
	videoStream = None

	def callback(self, data):
		# Pre-process input
		throttle = max(min(data.x, 0.9), -0.9)
		steering = -max(min(data.y, 0.9), -0.9)
		# Read from VideoStream
		frame = self.videoStream.read()
		img_path = self.recording_folder + 'img_' + str(self.img_count) + '.jpg'
		# Export results
		cv2.imwrite(img_path, frame)
		self.recording_csv.write(str(self.img_count) + '.jpg, ' + str(throttle) + ', ' + str(steering) + '\n')
		self.img_count += 1

	def __init__(self, path):
		# Export path
		self.path = path
		if not os.path.exists(self.path):
			os.makedirs(self.path)
		self.recording_csv = open(self.path + 'annotations.csv', 'a')

		# Initialize ROS Subscriber
		rospy.init_node('data_saver', anonymous=True)
		rospy.Subscriber('cmd', Point, self.callback)

		# Open Camera stream
		self.videoStream = WebcamVideoStream().start()
		time.sleep(1.0)
		rospy.spin()

	def __exit__(self, exc_type, exc_value, traceback):
		self.videoStream.stop()
		if(self.recording_csv is not None):
			self.recording_csv.close()
			self.recording_csv = None
