#!/usr/bin/env python
import roslib; roslib.load_manifest('formation_assignment')
import rospy
import irobot_mudd
from std_msgs.msg import *
from irobot_mudd.srv import *
from irobot_mudd.msg import *
#import Assignment
#import FormationSynthesis




def transition(time, next_state):
    """Transition function to help state machine."""
    
    #Check if wheels have dropped
    if D.data.wheeldropCaster:
        print "Wheel drop!"
        print "Shutting down..."
        Robo.move(0,0)
        Robo.mode = "ambivalent"
        Robo.status = "shutdown"
        rospy.signal_shutdown("robot picked up... so we're shutting down")

