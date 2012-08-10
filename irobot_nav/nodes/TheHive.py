#!/usr/bin/env python
import roslib; roslib.load_manifest('irobot_nav')
import rospy
from Data import Data
from Robot import Robot


#Initializes instance of data
D = Data()

#Initializes instance of robot
R = Robot()


def get_data_instance():
    """Returns reference to instance of data object when called."""
    return D

def get_robot_instance():
    """Returns reference to instance of robot object when called."""
    return R


