#!/usr/bin/env python
import roslib; roslib.load_manifest('robot_convoy')
import rospy
import irobot_mudd
import sensor_msgs.msg as sm
from irobot_mudd.srv import *
from irobot_mudd.msg import *
import math
import time
from Robot import Robot
import Navigator
import HandleData
import sys


################################################################################
###
### Antonella Wilby and Vivian Wehner
###
### This class module provides a finite state machine that controls both the
### leader and follower behavior for the robot convoy.  Robots are activated
### using 'A', and set to either leader or follower mode using '1' and '2',
### respectively.  Leader mode tracks a white line on the floor using the
### iRobot's infrared cliff sensors.  Follower mode tracks a previously
### thresholded color target.
###
################################################################################


#Get instances of data and robot from Navigator
D = Navigator.get_data_instance()
Robo = Navigator.get_robot_instance()


#################### ROBOT SPECIFIC FUNCTIONS #####################

def activate():
    """Activates robot, or after a deactivation, reactivates."""
    
    if Robo.state != "active":
        Robo.state = "active"
        print "Robot is now active"
    else:
        Robo.state = "not active"
        print "Robot is no longer active"
        Robo.speed = (0,0)
        Robo.move(0,0)
        HandleData.make_others_wait()



####################### BEGIN STATE MACHINE ########################

def transition(time, next_state):
    """Transition function to help state machine."""
    if str(D.robots[Robo.name]) == "stop":
        print "robot lost target"
        Robo.speed = (0,0)
        Robo.move(0,0)
    
    elif str(D.robots[Robo.name]) == "wait":
        print "robot waiting on other robots"
        Robo.speed = (0,0)
        Robo.move(0,0)
    
    elif Robo.state == "not active":
        print "robot has not been activated"
        Robo.speed = (0,0)
        Robo.move(0,0)
    
    if D.data.wheeldropCaster:
        print "Wheel drop!"
        print "Stopping..."
        Robo.speed = (0,0)
        Robo.move(0,0)
        HandleData.make_others_wait()
        #Robo.mode = "ambivalent"
        rospy.signal_shutdown("robot picked up... so we're shutting down")
     
    else:
        rospy.Timer(rospy.Duration(time), next_state, oneshot=True)
    

def state_start(mode, timer_event=None):
    """Starts either leader or follower behavior
    depending on what mode the robot is in."""
    HandleData.set_all("go")
    if mode == "leader":
        #Robo.mode = "leader"
        print "Leading the way!"
        transition(0.1, state_lead)
        
    elif mode =="follower":
        #Robo.mode = "follower"
        print "Following the leader."
        transition(0.1, state_follow)
    else:
        print "I'm feeling ambivalent right now."
        



### These functions define the leader behavior for the robot ###

def state_lead(timer_event=None):
    """Follows lines."""
    max = 80
    whiteThreshold = 500        #Set threshold for white sensor

    # print "Following a line..."
    if D.robots[Robo.name] == "0" or D.robots[Robo.name] == "wait":
        Robo.speed = (0,0)
        Robo.move(0,0)
        transition(0.1, state_lead)
    
    else:
        print "we move now?"
        Robo.speed = (max,max)
        Robo.move(max, max)           #Command robot to move forward
        
        if D.data.cliffFrontLeftSignal > whiteThreshold:
            Robo.speed = (-max,max)
            Robo.move(-max, max)
            
        elif D.data.cliffFrontRightSignal > whiteThreshold:
            Robo.speed = (max,-max)
            Robo.move(max,-max)
        
        elif D.data.cliffLeftSignal > whiteThreshold:
            Robo.speed = (-max,max)
            Robo.move(-max, max)
            
        elif D.data.cliffRightSignal > whiteThreshold:
            Robo.speed = (-max,max)
            Robo.move(-max, max)
        
        # Add check for running off a line- should set speed to zero if that is the case
        transition(0.1, state_lead)
       
       
            
### These functions define the follower behavior for the robot ###
    
def state_follow(timer_event=None):
    print "following"
    idealSize, constantOfP, maxSpeed, error = Robo.ideal_dist, 1/100.0, 110, 20
    newSpeed = constantOfP * (idealSize - D.target_size)
    if newSpeed > maxSpeed:
        newSpeed = maxSpeed
    elif newSpeed < -maxSpeed:
        newSpeed = -maxSpeed
        
    # If we see a target
    if D.target_size > D.min_T_size:
        
        # If it is about to leave the frame, emergency direction change!
        if D.target_coord[0] < 100 or D.target_coord[0] > 540:
            transition(0.1,state_turn_to_goal)
            
        #If we are alarmingly far from it, charge forward
        elif Robo.ideal_dist - D.target_size > (Robo.ideal_dist / 3.0):
            Robo.speed = (Robo.max_speed, Robo.max_speed)
            Robo.move(Robo.max_speed, Robo.max_speed)
            transition(0.1, state_follow)
        
        
        # If we see the target and it's not too far, check angles
        else:
            # If the target is pointed in the right direction we can move
            if abs(D.target_coord[0] - 320) < error:
                print "in the right direction too"
                
                # You hit the sweet spot, so don't mess it up.
                if abs(newSpeed) < 5:
                    Robo.speed = (0,0)
                    Robo.move(0,0)
                    transition(0.1, state_follow)
                    
                else:
                    # OK move now
                    Robo.speed = (int(newSpeed), int(newSpeed))
                    Robo.move(int(newSpeed), int(newSpeed))
                    transition(0.1, state_follow)
        
            # We see a target but aren't orientated correctly
            else:
                transition(0.1,state_turn_to_goal)
            
    # We don't see a target so we need to make everyone wait on us.
    else:
        Robo.speed = (0,0)
        Robo.move(0,0)
        HandleData.make_others_wait()
        transition(0.1, state_seek)
    

def state_seek(timer_event=None):
    # Still can't find target
    print "in state seek"
    if D.target_size < D.min_T_size:
        transition(0.1, state_seek)
    # We found the target!
    else:
        print "woot we found the target"
        HandleData.set_status(Robo.name, "go")
        D.robots[Robo.name] = "go"
        HandleData.start_up_who_can()
        transition(0.1, state_follow)
    
    
def state_turn_to_goal(timer_event=None):
    # Fields
    print "turning to goal"
    error, idealX, maxSpeed, porp_diff = 20, 320, 110, .25
    targetXDiff = D.target_coord[0] - idealX
    speed = targetXDiff * porp_diff
    
    # If target isn't a residual, turn if needed
    if  D.target_size > D.min_T_size:
        
        # If we hit the sweet spot, don't turn anymore
        if abs(speed) < 5:
            Robo.speed =(0,0)
            Robo.move(0,0)
            transition(0.1, state_follow)
            
        else:
            Robo.speed = (speed,-speed)
            Robo.move(speed,-speed)
            transition(0.1, state_follow)
            
    # Whelp, we lost the target
    else:
        Robo.speed =(0,0)
        Robo.move(0,0)
        HandleData.make_others_wait()
        transition(0.1, state_seek)




def state_stop():
    """Stops robot when commanded by keyboard (space bar).""" 
    print "Stopping..."
    Robo.speed = (0,0)
    Robo.move(0,0)
    #Robo.mode = "ambivalent"

