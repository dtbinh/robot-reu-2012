#!/usr/bin/env python
import roslib; roslib.load_manifest('robot_convoy')
import rospy
import cv
import cv_bridge
from std_msgs.msg import Int8


################################################################################
###
### Antonella Wilby and Vivian Wehner
###
### This node publishes incoming keyboard input from the windows in our GUI to
### a topic so incoming input can be handled.
### 
### This node is NOT WORKING and UNIMPLEMENTED as of 8/10/12.
###
################################################################################

def keyboard():
    pub = rospy.Publisher('keyboard', Int8)
    rospy.init_node('keyboard')
    
    while not rospy.is_shutdown():
        key_press = cv.WaitKey(5) & 255
        if key_press != 255:
            #str = "hello world %s"%rospy.get_time()
            rospy.loginfo(key_press)
            pub.publish(int(key_press))
        
        
if __name__ == '__main__':
    try:
        keyboard()
    except rospy.ROSInterruptException: pass
    
    
    