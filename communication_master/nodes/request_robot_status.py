#!/usr/bin/env python

import roslib; roslib.load_manifest('communication_master')
import rospy
import irobot_nav
from irobot_nav.msg import *
from HiveStatus import HiveStatus

hive = HiveStatus()

def handle_robot_update(update):
    print update



def subscribe_to_robots():
    numRobots = 1

    rospy.Subscriber('status_updates', Status, handle_robot_update)



def main():
    """Initializes node and sets everything up."""
    
    #Initialize our node
    rospy.init_node('request_robot_status')
    
    #Connect to each robot in the swarm
    subscribe_to_robots()

    #Run until program is stopped
    rospy.spin()



if __name__ == "__main__":
    main()
