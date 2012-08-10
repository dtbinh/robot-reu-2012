#!/usr/bin/env python
import roslib; roslib.load_manifest('irobot_nav')
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


#Get robot and data instances
D = TheHive.get_data_instance()
Robo = TheHive.get_robot_instance()

#Get hive status updates
hive = communication_client.get_hive_status()

#Robot status updates topic
status_updates = rospy.Publisher('status_updates', String)


#################### ROBOT SPECIFIC FUNCTIONS #####################

def activate():
    """Activates robot, or reactivates after a deactivation."""
    
    if Robo.state != "active":
        Robo.state = "active"
        print "Robot is now active"
    else:
        Robo.state = "not active"
        print "Robot is no longer active"



def check_hive_updates():
    """Checks for status updates from the hive."""
    
    #print "checking for hive updates..."
    #print hive.status
    return hive.status




####################### BEGIN STATE MACHINE ########################

def transition(time, next_state):
    """Transition function to help state machine."""

    #Check if wheels have dropped
    if D.data.wheeldropCaster:
        print "Wheel drop!"
        print "Shutting down..."
        Robo.move(0,0)
        Robo.mode = "ambivalent"
	Robo.status = "shutdown"
        rospy.signal_shutdown("robot picked up... so we're shutting down")
    
 

    #Check for incoming messages from the hive
    hiveCommand = check_hive_updates()

    Robo.hiveCommand = hiveCommand

    #if Robo.hiveCommmand == "stop":
    #	rospy.Timer(rospy.Duration(time), state_wait, oneshot=True)
    
    print Robo.curState

    #Publish robot status updates
    update = Robo.status
    rospy.loginfo(update)
    status_updates.publish(String(update))


    rospy.Timer(rospy.Duration(time), next_state, oneshot=True)

    

def state_start(mode, timer_event=None):
    """Starts either leader or follower behavior
    depending on what mode the robot is in."""

    if mode == "leader":
        Robo.mode = "leader"
        print "Leading the way!"
        Robo.curState = "state_lead"
        transition(0.1, state_lead)
        
    elif mode =="follower":
        Robo.mode = "follower"
        print "Following the leader."
        Robo.curState = "state_follow"
        transition(0.1, state_follow)
    else:
        print "I'm feeling ambivalent right now."
        


### These functions define the leader behavior for the robot ###

def state_lead(timer_event=None):
    """Follows lines."""

    #print Robo.status

    max = 80			#Maximum robot speed
    whiteThreshold = 500        #Set threshold for white sensor

    Robo.move(max, max)         #Command robot to move forward
        
    #if D.data.cliffFrontLeftSignal > whiteThreshold or D.data.cliffLeftSignal > whiteThreshold:
    #    print "turning left"
    #    Robo.move(-max, max)
            
    #elif D.data.cliffFrontRightSignal > whiteThreshold or D.data.cliffRightSignal > whiteThreshold:
    #    print "turning right"
    #    Robo.move(max,-max)
        
    #Check for a bump
    if D.data.bumpRight or D.data.bumpLeft:
	Robo.status = "help!"
        
    transition(0.1, state_lead)
       
            
### These functions define the follower behavior for the robot ###
    
def state_follow(timer_event=None):

    idealSize = 20000		#Ideal size of target in pixels
    constantOfP = 1/100.0	#Proportion for calculating speed based on target distance
    maxSpeed = 110		#Maximum robot speed
    error = 30			#???
    
    #Calculate new robot speed based on current distance from target
    newSpeed = int(constantOfP * (idealSize - D.target_size))

    #Robot is ideal distance away from target, try to maintain distance
    if D.target_size < idealSize + 1000 and D.target_size > idealSize - 1000:
        Robo.move(0,0)
        transition(0.1, state_follow)
    
    #If target is too small we must move closer,
    #but any target smaller than 200px is probably noise
    elif D.target_size > 200:

        #Cap the new speed at the maximum speed
        if newSpeed > maxSpeed:
            newSpeed = maxSpeed
        elif newSpeed < -maxSpeed:
            newSpeed = -maxSpeed
        
        #Now that proper speed is calculated, move robot    
        Robo.move(newSpeed, newSpeed)
        
        #If we are off center, adjust heading
        if abs(D.target_coord[0] - 320) > error:
            Robo.curState = "state_turn_to_goal"
            transition(0.1, state_turn_to_goal)
            
        #Otherwise, transition to update distance and speed
        else:
            transition(0.1, state_follow)

    #If we lose the target, enter the seek state
    #and tell the other robots to wait
    else:
        Robo.curState = "state_seek"
        transition(0.1, state_seek)


def state_seek(timer_event=None):
    """Enters seek pattern to try to reacquire target."""

    Robo.status = "stop"

    #If target is below a certain size, seek
    #Any targets picked up below this size are disregarded as noise
    if D.target_size < 200:
        Robo.move(50, -50)
	rospy.sleep(1)
	Robo.move(-50, 50)
	rospy.sleep(1)
        transition(0.1, state_seek)
    
    #We found the target! Let other robots know
    else:
        Robo.status = "ok"
        Robo.curState = "state_follow"
        transition(0.1, state_follow)
    
    
def state_turn_to_goal(timer_event=None):
    """Orients robot towards the goal."""

    error, idealX = 10, 320
    targetXDiff = D.target_coord[0] - idealX
    
    # If target isn't a residual
    if  D.target_size > 200:
        
	#Turn to goal
        if targetXDiff > error:
            #Robo.move(maxSpeed, maxSpeed - turnSpeed)
            Robo.move(50,20)
            transition(0.05, state_turn_to_goal)
        elif targetXDiff < -error:
            #Robo.move(maxSpeed - turnSpeed, maxSpeed)
            Robo.move(20,50)
            transition(0.05, state_turn_to_goal)
        
        #We are pointed in approximatly the correct direction!
        else:
            Robo.curState = "state_follow"
            transition(0.05, state_follow)
    
    #We lost the target
    else:
        Robo.curState = "state_seek"
	Robo.status = "stop"
        transition(0.1, state_seek)



def state_wait(timer_event=None):
    """Stops and waits for a command from the hive master."""
    
    print "Waiting for command..."

    if Robo.speed != 0:
	Robo.move(0, 0)

    transition(0.1, next_state)




def state_stop(timer_event=None):
    """Stops robot when commanded by keyboard (space bar).""" 
    print "Stopping..."
    Robo.move(0,0)
    #Robo.mode = "ambivalent"



####################### END STATE MACHINE ##########################
