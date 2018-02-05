#! /usr/bin/env python
import os
import pprint
import pygame
import rospy
from std_msgs.msg import String

class ModeManager(object):

    # Controller
    controller = None
    button_data = None
    # ROS
    pub = None
    mode = None

    def init(self):   
    	# Initialize joystick receiver using pygame
        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

    	# Initialize ROS Publisher
	self.pub = rospy.Publisher('mode', String, queue_size=10)
    	rospy.init_node('controller', anonymous=True)

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
    mm = ModeManager()
    mm.init()
    mm.listen()
