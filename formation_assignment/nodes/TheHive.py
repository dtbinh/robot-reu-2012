#!/usr/bin/env python
import roslib; roslib.load_manifest('formation_assignment')
import rospy
from Data import Data
from Robot import Robot



class Hive():
    def __init__(self):
        self.number = 4
        self.position1 = [0, 0]
        self.position2 = [5, 5]
        self.position3 = [-9, 8]
        self.position4 = [13, 20]


#Initializes instance of data
D = Data()

#Initializes instance of robot
R = Robot()

#Initializes instance of hive
H = Hive()


def get_data_instance():
    """Returns reference to instance of data object when called."""
    return D

def get_robot_instance():
    """Returns reference to instance of robot object when called."""
    return R

def get_hive_instance():
    """Returns reference to instance of hive status when called."""
    return H


