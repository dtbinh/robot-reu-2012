#!/usr/bin/env python
import roslib; roslib.load_manifest('formation_assignment')
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

#Get data and robot instances
D = TheHive.get_data_instance()
R = TheHive.get_robot_instance()


#Get hive status from the communication node
#hive = communication_client.get_hive_status()


def initialize_interface():
    """Initializes all windows needed by our program."""
    
    #Create window to monitor robot status
    cv.NamedWindow('Monitor')
    cv.MoveWindow('Monitor', 0, 0)
    D.size = (100,100)
    D.dummy = cv.CreateImage(D.size, 8, 1)
    D.image = cv.Zero(D.dummy)
    cv.ShowImage('Monitor', D.image)



def get_services():
    """Connects to all ROS services needed by this node."""
    
    #Obtain the tank service
    print "Connecting to tank service..."
    rospy.wait_for_service('tank')      #Won't continue until the "tank" service is connected
    
    #Obtain the master message service
    print "Connecting to communication master..."
    #rospy.wait_for_service('communication_master')    #Connects to the "hive mind"
    
    

def get_topics():
    """Subscribes to all ROS topics needed by this node."""
    
    #Subscribe to the sensorPacket topic
    rospy.Subscriber('sensorPacket', SensorPacket, HandleData.handle_sensor_data)
       
    #Subscribe to the hive commands topic
    rospy.Subscriber('commands', String, HandleData.handle_hive_commands)



def main():
    """Initializes node and subscribes to services."""
    
    #Initialize ROS node
    rospy.init_node('formation_control')
    
    #Initialize our program interface
    initialize_interface()
    
    #Connect to ROS services
    get_services()
    
    #Subscribe to ROS topics
    get_topics()
    
    #Run until something stops the program
    rospy.spin()



if __name__ == "__main__":
    main()
