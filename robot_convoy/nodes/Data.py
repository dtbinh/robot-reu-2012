#!/usr/bin/env python
import roslib; roslib.load_manifest('robot_convoy')
import rospy
import irobot_mudd
import cv_bridge
import cv
import sensor_msgs.msg as sm
from std_msgs.msg import String
from irobot_mudd.srv import *
from irobot_mudd.msg import *
import ImageProcessing


################################################################################
###
### Antonella Wilby and Vivian Wehner
###
### This class stores image and threshold information from OpenCV, information
### created and used by the program's GUI, and data from the robot's sensors.
###
################################################################################

class Data:
    def __init__(self):
        self.thresholds = {'low_red':0, 'high_red':255,\
                       'low_green':0, 'high_green':255,\
		       'low_blue':0, 'high_blue':255, \
                       'low_hue':0, 'high_hue':255, \
                       'low_sat':0, 'high_sat':255, \
                       'low_val':0, 'high_val':255}

	self.mouse_down = False
	self.stopped = False

	#Target coordinate not set yet
	self.target_coord = (-10, -10)
	self.last_target_coord = (-10, -10)
	self.target_size = 0
    
	# Highlighted area does not exist yet
	self.down_coord = (-1,-1)
	self.up_coord = (-1,-1)
	self.sections = []
	self.mode = "clear"
	
	#Set position for 4 angle-calculating points
	self.p1, self.p2, self.p3, self.p4 = (300,240), (340,240), (320,220), (320,260)
	
	# We have not created our "scratchwork" images yet
	self.created_images = False
    
	# Variable for key presses - set it to something that could not have been pressed
	self.last_key_pressed = 255
    
	# The current image we want to display in the threshold window
	self.current_threshold = "threshed_image"
    
	# Create a connection to the Kinect
	self.bridge = cv_bridge.CvBridge()
	
	self.threshed_image = False
	
	
	self.font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1)
	self.white = cv.RGB(255, 255, 255)
	self.black = cv.RGB(0, 0, 0)
	#self.data, self.decisionData = {}, {}
	
	#Haven't create range image yet
	self.range_image = False
	
	self.loaded_threshold = False
	
	# Message about other robots that we know
	self.min_T_size = 200
	self.robots = {"r1":"ok", "r2":"ok", "r3":"ok"}
	


    def __repr__(self):
        return str(vars(self))
	
    