#!/usr/bin/env python

import roslib; roslib.load_manifest('irobot_nav')
import rospy
from std_msgs.msg import *
from irobot_nav.msg import *
import TheHive


robot = TheHive.get_robot_instance()


def publish_robot_status():
    status_updates = rospy.Publisher('status_updates', String)
    while not rospy.is_shutdown():
        update = robot.status
        rospy.loginfo(update)
        status_updates.publish(String(update))
        rospy.sleep(0.5)


    


def main():
    """Initializes node, creates services, and starts publishing topics."""
    
    #Initialize master communication node
    rospy.init_node('robot_status')
    
    #Publish robot's status updates
    publish_robot_status()

    #Run until program is stopped
    rospy.spin()



if __name__ == "__main__":
    main()
