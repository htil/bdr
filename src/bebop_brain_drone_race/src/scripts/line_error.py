#!/usr/bin/env python
# This package is where all openCV-related things will happen.
from sensor_msgs.msg import Image

def line_error():
	# OpenCV calculation
	
rospy.Subscriber("/bebop/image_raw", Image, line_error)