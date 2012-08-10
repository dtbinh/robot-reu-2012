#!/usr/bin/env python
import roslib; roslib.load_manifest('irobot_nav')
import rospy
import irobot_mudd
import sensor_msgs.msg as sm
from irobot_mudd.srv import *
from irobot_mudd.msg import *


class Robot:
    def __init__(self):
        self.speed = 0
        self.state = "dormant"
        self.mode = "ambivalent"
	self.curState = ""
        self.max_speed = 100
        self.tank = rospy.ServiceProxy('tank',Tank)
        self.stop = False
        self.hiveCommand = "none"
        
        #Status for message generation
        self.status = "ok"
        
    
    def move(self, left, right):
        """Calls the tank function to move the robot."""
        if (left,right) == self.speed:
            return
        elif self.state == "active":   
            try:
                self.tank(left, right)
                self.speed = (left + right) / 2
            except:
                print "There was a problem"
                print "error", sys.exc_info()[0]

            
        elif self.state == "not active":
            self.tank(0,0)
            self.speed = (0,0)
            print "Robot is not active"
                        
            
