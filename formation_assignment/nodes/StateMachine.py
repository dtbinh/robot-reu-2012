#!/usr/bin/env python
import roslib; roslib.load_manifest('formation_assignment')
import rospy
import irobot_mudd
import sensor_msgs.msg as sm
from std_msgs.msg import String
from irobot_mudd.srv import *
from irobot_mudd.msg import *
import math
import time
import TheHive
import communication_client
import MultiRobotAssignment
import FormationSynthesis

#Get Robot and data instances
D = TheHive.get_data_instance()
R = TheHive.get_robot_instance()

#Get hive status updates
hive = communication_client.get_hive_status()

#Robot status updates topic
status_updates = rospy.Publisher('status_updates', String)



def check_hive_updates():
    """Checks for status updates from the hive."""
    
    #print "checking for hive updates..."
    #print hive.status
    #return hive.status
    pass


def check_hive_command():
    """Checks for commands from the hive master."""
    
    if R.hiveCommand == "start":
	transition(0.1, state_wait_for_formation)
    elif R.hiveCommand == "stop":
	transition(0.1, state_stop)
    elif R.hiveCommand == "pause":
	transition(0.1, state_wait_for_start)
    elif R.hiveCommand == "line":
	transition(0.1, state_form_line)
    elif R.hiveCommand == "square":
	transition(0.1, state_form_square)

    

def transition(time, next_state):
    """Transition function to help state machine."""

    #Check if wheels have dropped
    if D.data.wheeldropCaster:
        print "Shutting down..."
        R.move(0,0)
	R.status = "shutdown"
	rospy.signal_shutdown("Robot picked up... so we're shutting down")
    
 

    #Check for incoming messages from the hive
    hiveCommand = check_hive_updates()

    R.hiveCommand = hiveCommand

    #if R.hiveCommmand == "stop":
    #	rospy.Timer(rospy.Duration(time), state_wait, oneshot=True)

    #Publish Robot status updates
    update = R.status
    rospy.loginfo(update)
    status_updates.publish(String(update))
    
    print "checking hive command"
    check_hive_command()


    rospy.Timer(rospy.Duration(time), next_state, oneshot=True)



def state_start(timer_event=None):
    """Starts assignment and formation control algorithm."""

    print "Starting formation control..."
    
    transition(0.1, state_wait_for_formation)
    
    
def state_wait_for_start(timer_event=None):
    """Waits for a start command from the hive master."""
    
    print "Waiting for command..."
    
    if R.hiveCommand == "start":
	transition(0.1, state_start)
    else:
	transition(0.1, state_wait_for_start)
	
	
def state_wait_for_formation(timer_event=None):
    """After formation control is started, waits for a command to form
    a specific formation.
    """
    
    print "Waiting for formation command..."
    
    transition(0.1, state_wait_for_formation)


def state_stop(timer_event=None):
    """Stops Robot when commanded by keyboard (space bar).""" 
    print "Stopping..."
    R.move(0,0)
    
    
    
    
########################  FORMATION-SPECIFIC STATES  ###########################

def state_form_line(timer_event=None):
    """Initiates assignment calculation and formation synthesis calculation for
    formation of a line of robots.
    """
    
    pass


def state_form_square(timer_event=None):
    """Initiates assignment calculation and formation synthesis calculation for
    formation of group of robots into a square.
    """
    
    pass





