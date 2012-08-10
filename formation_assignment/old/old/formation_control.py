#!/usr/bin/env python
import roslib; roslib.load_manifest('formation_assignment')
import rospy
import irobot_mudd
import cv_bridge
import cv
from std_msgs.msg import *
from irobot_mudd.srv import *
from irobot_mudd.msg import *
import StateMachine
from Robot import Robot
from Data import Data

#Initialize instance of robot
R = Robot()

#Initialize instance of robot class
D = Data()



def get_services():
    """Connects to all ROS services needed by this node."""
    
    #Obtain tank service
    print "Connecting to tank service..."
    #rospy.wait_for_service('tank')
    
    #Obtain communication service
    print "Connecting to communication master..."
    #rospy.wait_for_service('communication_master')
    
    
    
def get_topics():
    """Subscribes to all ROS topics needed by this node."""
    
    #Subscribe to the incoming SensorPackets from the sensorPacket topic
    #rospy.Subscriber('sensorPacket', SensorPacket, handle_sensor_data)
    
    pass
    
def initialize_interface():
    """Initializes the windows needed by this node."""
    
    print "Initializing..."
    
    #Create window to show robot status
    cv.NamedWindow('Monitor')
    cv.MoveWindow('Monitor', 0, 0)
    D.dummy = cv.CreateImage(D.size, 8, 1)
    cv.ShowImage('Monitor', cv.Zero(D.dummy))



def main():
    """Sets up ROS node and anything else needed for our program."""
    
    #Initialize ROS node
    rospy.init_node('formation_control', anonymous=True)
    
    #Initialize our program interface
    initialize_interface()
    
    #Get ROS services
    get_services()
    
    #Subscribe to data topics
    get_topics()
    
    #Run until program is stopped
    rospy.spin()



if __name__ == "__main__":
    main()