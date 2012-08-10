#!/usr/bin/env python
import roslib; roslib.load_manifest('formation_assignment')
import rospy
import irobot_mudd
from irobot_mudd.srv import *
from irobot_mudd.msg import *




class Robot:
    """Defines the robot class."""
    
    def __init__(self):
        self.speed = (0, 0)
        self.status = "ok"              #
        self.assignment = 0
        self.state = "dormant"          #Set to active by the master when commanded      
        
        #Set up call for tank service
        self.tank = rospy.ServiceProxy('tank', Tank)
        
        
    def move(self, left, right):
        """Calls tank service to move robot at the desired speed."""
        
        try:
            self.tank(left, right)
            self.speed = (left, right) 
        except:
            print "There was an error calling the tank service."
            print "Error:", sys.exc_info()[0]