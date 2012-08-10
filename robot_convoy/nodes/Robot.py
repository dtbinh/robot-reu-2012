#!/usr/bin/env python
import roslib; roslib.load_manifest('robot_convoy')
import rospy
import irobot_mudd
import sensor_msgs.msg as sm
from irobot_mudd.srv import *
from irobot_mudd.msg import *
from Data import Data
import math
import sys
import traceback


################################################################################
###
### Antonella Wilby and Vivian Wehner
###
### This class defines all properties of the robot and defines any functions
### that can be called on the robot.
###
################################################################################


class Robot:
    def __init__(self):
        self.speed = (0, 0)
        self.state = "not active"  
        self.curState = ""         
        self.max_speed = 100
        self.ideal_dist = 15000
        self.tank = rospy.ServiceProxy('tank',Tank)
        self.stop = False
        self.name = "r1"
        self.num = 1
    
    def move(self, left, right):
        """Calls the tank service to move the robot given left and right
           wheel speeds as input.
        """
        if self.state == "active":
            try:
                self.tank(left, right)
                self.speed = (left, right)
            except:
                print "There was a problem"
                print "error", sys.exc_info()[0]
                tb = traceback.format_exc()
                # if it's more serious...
                # then here we should restart/reset the driver...
                print "Traceback is", tb
            
        elif self.state == "not active":
            self.tank(0,0)
            self.speed = (0,0)
            print "Robot is not active"
            