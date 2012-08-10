#!/usr/bin/env python

import roslib; roslib.load_manifest('irobot_nav')
import rospy


class HiveStatus():
    def __init__(self):
	self.status = ""
	self.warning = ""
	self.command = ""



