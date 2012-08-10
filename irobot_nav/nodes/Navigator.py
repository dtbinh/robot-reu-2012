#!/usr/bin/env python
import roslib; roslib.load_manifest('irobot_nav')
import rospy
import irobot_mudd
import cv_bridge
import cv
import sensor_msgs.msg as sm
from std_msgs.msg import String
from irobot_mudd.srv import *
from irobot_mudd.msg import *
import TheHive
import HandleData
import Interface

#Get data and robot instances
D = TheHive.get_data_instance()
R = TheHive.get_robot_instance()


#Get hive status from the communication node
#hive = communication_client.get_hive_status()



########################## INITIALIZATION FUNCTIONS ############################


def dummy_window():
    cv.NamedWindow('keyboard')
    cv.MoveWindow('keyboard', 0, 0)
    D.size = (100,100)
    D.dummy = cv.CreateImage(D.size, 8, 1)
    D.created_images = True
    cv.ShowImage('keyboard', cv.Zero(D.dummy))


def initialize():
    """Initializes interface and callback functions."""
    
    #Initialize all the windows and other things needed for the program interface
    #Set up the windows containing the image from the kinect camera, the altered 
    #	threshold image the threshold sliders, and the kinect range image.  
    #Interface.initialize(D)


    #Sets up a dummy window for taking keyboard input without a kinect
    dummy_window()

    #Set the method to handle incoming mouse data in the Image window
    #cv.SetMouseCallback('Image', HandleData.mouseImage, None)
    
    #Set the method to handle incoming mouse data in the Range window
    #cv.SetMouseCallback('Range', HandleData.mouseRange, None)
    

######################## END INITIALIZATION FUNCTIONS ##########################



def ros_services():
    """Sets data services."""
    
    #Obtain the tank service
    rospy.wait_for_service('tank')      #Won't continue until the "tank" service is connected
    
    #Obtain the song service
    rospy.wait_for_service('song')      #Won't continue until the "song" service is connected
    
    #Obtain the master message service
    #rospy.wait_for_service('communication_master')    #Connects to the "hive mind"
    
    

def ros_topics():
    """Subscribes to data topics."""
    
    #Subscribe to the sensorPacket topic
    rospy.Subscriber('sensorPacket', SensorPacket, HandleData.handle_sensor_data)
    
    #Subscribe to the image color topic
    #rospy.Subscriber('/camera/rgb/image_color', sm.Image, HandleData.handle_image_data)
    
    #Subscribe to the range image topic
    #rospy.Subscriber('/camera/depth/image', sm.Image, HandleData.handle__range_data)
   


def main():
    """Initializes node and subscribes to services."""
    
    #Initialize our node
    rospy.init_node('robot_nav')
    
    #Initialize everything we need for the program
    initialize()
    
    #Subscribe to ROS services
    ros_services()
    
    #Subscribe to ROS topics
    ros_topics()
    
    #Run until something stops the program
    rospy.spin()



if __name__ == "__main__":
    main()
