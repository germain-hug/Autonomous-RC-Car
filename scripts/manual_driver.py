#! /usr/bin/env python

"""
- Activated when driving in manual mode
- Press 'O' to record
- Press 'square' to stop recording
- 

"""

import os
import pprint
import pygame
import rospy
from std_msgs.msg import String


class ManualDriver(object):

    controller = None
    button_data = None
    mode = None

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
	rospy.spin()

    # Listen for PS4 Controller inputs
    def listen(self):
        if not self.button_data:
            self.button_data = {}
            for i in range(self.controller.get_numbuttons()):
                self.button_data[i] = False
        
        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if(event.button==1):
			rospy.loginfo("Switching to manual...")
			self.pub.publish("manual")
		    elif(event.button==3):
			rospy.loginfo("Switching to auto...")
			self.pub.publish("auto")



if __name__ == "__main__":
    manual_driver = ManualDriver()
    manual_driver.init()
    manual_driver.listen()
