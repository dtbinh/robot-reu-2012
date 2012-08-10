#!/usr/bin/env python
import roslib; roslib.load_manifest('communication_master')
import rospy
from std_msgs.msg import *
from communication_master.srv import *
from HiveStatus import HiveStatus


#Initialize the hive
hive = HiveStatus()


def publish_hive_updates():
    buzz = rospy.Publisher('hive_buzz', String)
    while not rospy.is_shutdown():
	update = "status:%s"%hive.status
	rospy.loginfo(update)
	buzz.publish(String(update))
	rospy.sleep(0.5)
    


def handle_hive_status_requests(req):
    """Handles any requests for the global status of the hive."""
    
    status = hive.status
    return StatusResponse(status)



def hive_status_updates():
    """Service that sends the global status of the hive upon request."""
    
    rospy.Service('hive_status_updates', Status, handle_hive_status_requests)




def main():
    """Initializes node and starts any services, publishers, and subscribers."""

    #Initialize the master communication node
    rospy.init_node('communication_master')

    #Start master communication service
    #hive_status_updates()

    #Start publishing robot messages
    try:
	publish_hive_updates()
    except rospy.ROSInterruptException:
	pass

    #Run until the program is stopped
    rospy.spin()




if __name__ == "__main__":
    main()
