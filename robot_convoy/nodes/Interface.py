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


################################################################################
###
### Antonella Wilby and Vivian Wehner
###
### This module creates the windows for the GUI interface.
###
################################################################################




def initialize(D):
    """Initializes everything needed for the program interface."""
    
    create_windows()
    
    #create_sliders(D)


def create_windows():
    """Set up the windows containing the image from the kinect camera, the thresholded
    image, the threshold sliders, and the kinect range image.
    
    """
    
    #Create window to show image from Kinect camera
    cv.NamedWindow('Image')
    cv.MoveWindow('Image', 0, 0)
    
    #Create window to show thresholded image
    cv.NamedWindow('Threshold')
    cv.MoveWindow('Threshold', 640, 0)
    
    #Create window to store threshold sliders
    #cv.NamedWindow('Sliders')
    #cv.MoveWindow('Sliders', 650, 0)
    
    #Create window to show range image from kinect depth sensor
    #cv.NamedWindow('Range')
    #cv.MoveWindow('Range', 980, 0)


def create_sliders(D):
    """Initializes all the sliders within the sliders window."""
    
    #Create the Red/Green/Blue sliders
    cv.CreateTrackbar('low_red', 'Sliders', D.thresholds['low_red'], 255, \
                          lambda x: change_slider(D, 'low_red', x) )
    cv.CreateTrackbar('high_red', 'Sliders', D.thresholds['high_red'], 255, \
                          lambda x: change_slider(D, 'high_red', x) )
    cv.CreateTrackbar('low_green', 'Sliders', D.thresholds['low_green'], 255, \
                          lambda x: change_slider(D, 'low_green', x) )
    cv.CreateTrackbar('high_green', 'Sliders', D.thresholds['high_green'], 255, \
                          lambda x: change_slider(D, 'high_green', x) )
    cv.CreateTrackbar('low_blue', 'Sliders', D.thresholds['low_blue'], 255, \
			  lambda x: change_slider(D, 'low_blue', x))
    cv.CreateTrackbar('high_blue', 'Sliders', D.thresholds['high_blue'], 255, \
			  lambda x: change_slider(D, 'high_blue', x))

    #Create the Hue/Saturation/Value sliders
    cv.CreateTrackbar('low_hue', 'Sliders', D.thresholds['low_hue'], 255, \
                          lambda x: change_slider(D, 'low_hue', x))
    cv.CreateTrackbar('high_hue', 'Sliders', D.thresholds['high_hue'], 255, \
                          lambda x: change_slider(D, 'high_hue', x))
    cv.CreateTrackbar('low_sat', 'Sliders', D.thresholds['low_sat'], 255, \
                          lambda x: change_slider(D, 'low_sat', x))
    cv.CreateTrackbar('high_sat', 'Sliders', D.thresholds['high_sat'], 255, \
                          lambda x: change_slider(D, 'high_sat', x))
    cv.CreateTrackbar('low_val', 'Sliders', D.thresholds['low_val'], 255, \
                          lambda x: change_slider(D, 'low_val', x))
    cv.CreateTrackbar('high_val', 'Sliders', D.thresholds['high_val'], 255, \
                          lambda x: change_slider(D, 'high_val', x))


def change_slider(D, name, new_threshold):
    """Changes the slider values given the name of the slider and the new value."""

    D.thresholds[name] = new_threshold


