#! /usr/bin/env python

import os
import time
import rospy
import imutils

from std_msgs.msg import String
from geometry_msgs.msg import Point
from imutils.video import WebcamVideoStream

class SelfDriver(object):
    """ ** Self Driving Node **
    """
	def callback(self, data):
        # Callback method
		self.curr_mode = data.data
		rospy.loginfo("Switched to mode: " + self.curr_mode)

	def __init__(self):
		# Initialize ROS Subscriber and Publisher
		rospy.init_node('self_driver', anonymous=True)
		rospy.Subscriber('mode', String, self.callback)
		self.pub = rospy.Publisher('cmd', Point, queue_size=1)
        # Open Camera stream
		self.videoStream = WebcamVideoStream().start()
		time.sleep(1.0)
        # Initialize network
        frame = self.videoStream.read()
        self.model = network(frame.shape)
        self.model.load_model("models/model.h5")

	# Listen for Controller inputs
	def listen(self):
		# Main listening loop
		while not rospy.is_shutdown():
			if self.curr_mode =='auto':
                # Read from VideoStream
        		frame = self.videoStream.read()
                # Generate steering / throttle prediction
                preds = self.model.predict(frame)
                # TODO -> extract steering / throttle
                self.pub.publish(Point(throttle,steering,0.0))


if __name__ == "__main__":
	self_driver = SelfDriver()
	self_driver.listen()
