#!/usr/bin/env python

import roslib; roslib.load_manifest('communication_master')
import rospy


class HiveStatus():
    def __init__(self):
        self.status = "ok"
	self.warning = "none"
	self.command = "none"

    def get_status():
        return self.status


    def __repr__(self):
        return str(vars(self))
