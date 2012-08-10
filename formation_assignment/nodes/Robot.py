#!/usr/bin/env python
import roslib; roslib.load_manifest('formation_assignment')
import rospy
import irobot_mudd
import sensor_msgs.msg as sm
from irobot_mudd.srv import *
from irobot_mudd.msg import *


class Robot:
    """Defines the Robot class."""
    
    def __init__(self):
        self.speed = 0
	self.assignment = 0
        self.state = "dormant"		#Set to active by the master when commanded
	self.converged = False
	self.position = (0, 0)
        
	#Set up call for tank service
	self.tank = rospy.ServiceProxy('tank',Tank)
        
	#Set up fields for communication with hive master
	self.status = "ok"
	self.hiveCommand = "none"
        
        
    
    def move(self, left, right):
        """Calls tank service to move robot at the desired speed."""

	try:
	    self.tank(left, right)
	    self.speed = (left + right) / 2
	except:
	    print "There was an error calling the tank service."
	    print "Error:", sys.exc_info()[0]
