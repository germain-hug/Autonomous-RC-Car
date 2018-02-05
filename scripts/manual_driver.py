#! /usr/bin/env python

"""
- Activated when driving in manual mode
- Press 'O' to record
- Press 'square' to stop recording
- 

"""

import os
from datetime import timedelta, date
import time
import pygame
import rospy
from std_msgs.msg import String


class ManualDriver(object):

    controller = None
    button_data = None
    mode = 'manual'
    recording = False 
    recording_folder = None

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

    # Listen for Controller inputs
    def listen(self):
        if not self.button_data:
            self.button_data = {}
            for i in range(self.controller.get_numbuttons()):
                self.button_data[i] = False
        
	while not rospy.is_shutdown():

	    if self.mode =='manual':

		for event in pygame.event.get():
			if event.type == pygame.JOYBUTTONDOWN:

				# ---- Starting Recording ----
				if(event.button==0):
					rospy.loginfo("Starting recording...")
					self.recording = True
					self.recording_folder = '../data/' + str(date.today()) + "-" + str(time.strftime("%H-%M-%S"))
					print self.recording_folder
					# TODO: mkdir, etc...

				# ---- Stopping Recording ----
				elif(event.button==2):
					rospy.loginfo("Stopping recording...")
					self.recording = False


if __name__ == "__main__":
    manual_driver = ManualDriver()
    manual_driver.init()
    manual_driver.listen()
