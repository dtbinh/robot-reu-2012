#!/usr/bin/env python
import roslib; roslib.load_manifest('formation_assignment')
import rospy
import irobot_mudd
import cv_bridge
import cv
import sensor_msgs.msg as sm
from std_msgs.msg import String


################## BEGIN DATA HANDLING FUNCTIONS ###################

def handle_hive_updates(data):
    """Handle_sensor_data is called every time the robot gets a new sensorPacket."""
   
    print "handling messages"

    
