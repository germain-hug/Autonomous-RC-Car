#! /usr/bin/env python

"""
- Activated when driving in manual mode
- Press 'O' to record
- Press 'square' to stop recording
- 

"""

import os
import time
import pygame
import rospy
from datetime import timedelta, date
from std_msgs.msg import String
from geometry_msgs.msg import Point

class ManualDriver(object):

    controller = None
    axis_data = None
    button_data = None
    hat_data = None
    mode = 'manual'
    recording = False 
    recording_folder = None
    pub = None

    def callback(self, data):
	self.mode = data.data
	rospy.loginfo("Switched to mode: " + self.mode)

    def init(self):   
    	# Initialize joystick receiver using pygame
        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

    	# Initialize ROS Subscriber
	rospy.init_node('manual_driver', anonymous=True)
    	rospy.Subscriber('mode', String, self.callback)
        self.pub = rospy.Publisher('cmd', Point, queue_size=1)

    # Listen for Controller inputs
    def listen(self):

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
        
	while not rospy.is_shutdown():

	    if self.mode =='manual':

		for event in pygame.event.get():

			if event.type == pygame.JOYBUTTONDOWN:
				# Starting Recording
				if(event.button==0):
					rospy.loginfo("Starting recording...")
					self.recording = True
					self.recording_folder = '../data/' + str(date.today()) + "-" + str(time.strftime("%H-%M-%S"))
					print self.recording_folder
					# TODO: mkdir, etc...

				# Stopping Recording
				elif(event.button==2):
					rospy.loginfo("Stopping recording...")
					self.recording = False
			# Acceleration and Steering
			elif event.type == pygame.JOYAXISMOTION:
                    		self.axis_data[event.axis] = round(event.value,2)
				msg = Point(round(0.5 + self.axis_data[4] / 2.0, 5),round(self.axis_data[0], 5), 0.0)
				self.pub.publish(msg)


if __name__ == "__main__":
    manual_driver = ManualDriver()
    manual_driver.init()
    manual_driver.listen()
