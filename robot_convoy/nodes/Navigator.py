#!/usr/bin/env python
import roslib; roslib.load_manifest('robot_convoy')
import rospy
import irobot_mudd
import cv_bridge
import cv
import time
import threading
import sensor_msgs.msg as sm
from std_msgs.msg import Int8
from irobot_mudd.srv import *
from irobot_mudd.msg import *
import Interface
import HandleData
from Data import Data
from Robot import Robot



################################################################################
###
### Antonella Wilby and Vivian Wehner
###
### This module starts the ROS node navigator, subscribes to all publishers and
### connects to all services required, initializes the GUI interface, and
### initializes instances of the Data class and the Robot class.
###
################################################################################


#Initialize instance of data class
D = Data()

#Initialize instance of robot class
R = Robot()

def get_data_instance():
    """Returns reference to instance of data class to any module that needs it."""
    return D

def get_robot_instance():
    """Returns reference to instance of robot class to any module that needs it."""
    return R


#################### INITIALIZATION FUNCTIONS ######################

def initialize():
    """Initializes interface, images, and other global variables we need."""
    
    #Initialize all the windows and other things needed for the program interface
    #Set up the windows containing the image from the kinect camera, the altered 
    #	threshold image the threshold sliders, and the kinect range image.  
    Interface.initialize(D)

    #Set the method to handle incoming mouse data in the Image window
    cv.SetMouseCallback('Image', HandleData.mouseImage, None)
    
    #Set the method to handle incoming mouse data in the Range window
    cv.SetMouseCallback('Range', HandleData.mouseRange, None)



################## END INITIALIZATION FUNCTIONS ####################


def ros_services():
    """Sets data services and subscribes to data."""

    #Obtain the tank service
    rospy.wait_for_service('tank') 	#Won't continue until the "tank" service is on

    #Obtain the song service
    rospy.wait_for_service('song') 	#Won't continue until the "song" service is on

    #Subscribe to the sensorPacket topic
    rospy.Subscriber('sensorPacket', SensorPacket, HandleData.handle_sensor_data)

    #Subscribe to the image_color topic
    rospy.Subscriber('/camera/rgb/image_color', sm.Image, HandleData.handle_image_data)
     
    #Subscribe to the depth image topic
    #rospy.Subscriber('/camera/depth/image', sm.Image, HandleData.handle_range_image)

    #Subscribe to the incoming keyboard commands topic
    #rospy.Subscriber('keyboard', Int8, HandleData.handle_keyboard_data)


def conti_query():
    while True:
	time.sleep(0.05)
	HandleData.get_status()


def main():
    """Initializes everything we need for the program."""
    
    #Initialize ROS node
    rospy.init_node('navigator')

    #Initialize all the global variables
    initialize()

    #Subscribe to ROS services and data
    ros_services()
    
    #Continously check for status changes
    myT = threading.Thread(target=conti_query)
    myT.setDaemon(True)
    myT.start()
	
    #Run until program is stopped
    rospy.spin()



if __name__ == "__main__":
    main()

