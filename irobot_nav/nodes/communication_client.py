#!/usr/bin/env python

import roslib; roslib.load_manifest('irobot_nav')
import rospy
from std_msgs.msg import *
from irobot_nav.srv import *
from HiveUpdates import HiveStatus


hive = HiveStatus()

def get_hive_status():
    """Returns reference to hive status class whenever requested."""
    return hive



def request_hive_status(req):
    """Request for status updates from the hive."""
    
    #rospy.wait_for_service('hive_status_updates')
    
    try:
        request_hive_status = rospy.ServiceProxy('hive_status_updates', Status) #try persistent=True
        response = request_hive_status(req)
	hive.status = response
	return response.acknowledgment
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    except:
        print "Failed to get hive status."


if __name__ == "__main__":
    
    print "Connecting to the hive mind..."

    rospy.wait_for_service('hive_status_updates')

    while not rospy.is_shutdown():
        rospy.sleep(0.5)
        req =  "status?"
        print "Requesting status from the hive..."
        print "status is: %s"%(request_hive_status(req))


