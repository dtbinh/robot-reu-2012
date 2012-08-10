#!/usr/bin/env python
import roslib; roslib.load_manifest('irobot_mudd')
import rospy
import cv
import cv_bridge
from std_msgs.msg import Int8


################################################################################
###
### Antonella Wilby and Vivian Wehner
###
### This node publishes incoming mouse input from the windows in our GUI to
### a topic so incoming input can be handled.
### 
### This node is NOT WORKING and UNIMPLEMENTED as of 8/10/12.
###
################################################################################

def mouse():
    pub = rospy.Publisher('mouse', Int8)
    rospy.init_node('mouse')
    
    while not rospy.is_shutdown():
        pass
        
        
if __name__ == '__main__':
    try:
        mouse()
    except rospy.ROSInterruptException: pass
    
    
    