#! /usr/bin/env python
import os
import pprint
import pygame
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point

class ModeManager(object):

	# Bluetooth controller
	controller = None
	button_data = None

	# ROS publishers
	mode_pub = None
	cmd_pub = None
	mode = None

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

		# Initialize ROS Publisher
		self.mode_pub = rospy.Publisher('mode', String, queue_size=10)
		self.cmd_pub  = rospy.Publisher('cmd', Point, queue_size=1)
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
						# Start by stopping the car
						self.cmd_pub.publish(Point(0.0,0.0, 0.0))
						rospy.loginfo("Switching to manual...")
						self.mode_pub.publish("manual")
					elif(event.button==3):
						# Start by stopping the car
						self.cmd_pub.publish(Point(0.0,0.0, 0.0))
						rospy.loginfo("Switching to auto...")
						self.mode_pub.publish("auto")


if __name__ == "__main__":
    mm = ModeManager()
    mm.listen()
