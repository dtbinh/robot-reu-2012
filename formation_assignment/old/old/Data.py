#!/usr/bin/env python
import roslib; roslib.load_manifest('formation_assignment')
import rospy
import irobot_mudd
import cv_bridge
import cv



class Data:
    """Defines a class that stores all data needed by a robot."""
    
    def __init__(self):
        self.size = (200, 200)