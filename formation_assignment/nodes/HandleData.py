#!/usr/bin/env python
import roslib; roslib.load_manifest('formation_assignment')
import rospy
import irobot_mudd
import cv_bridge
import cv
import sensor_msgs.msg as sm
from std_msgs.msg import String
from irobot_mudd.srv import *
from irobot_mudd.msg import *
import TheHive
import StateMachine


#Get data and robot instances
D = TheHive.get_data_instance()
R = TheHive.get_robot_instance()



def handle_sensor_data(data):
    """Handle_sensor_data is called every time the robot gets a new sensorPacket."""

    #Store incoming data in the Data object
    D.data = data

    #Check for a bump
    if data.bumpRight or data.bumpLeft:
        print "Bumped!"


    #Check if play button was pressed	
    if data.play:
	print "Stopping..."
	StateMachine.state_stop()
	rospy.signal_shutdown("play button pressed")

    #Check key presses
    key_press = cv.WaitKey(5) & 255
    if key_press != 255:
    	check_key_press(D, key_press)	

    #Display robot updates in Monitor window
    draw_on_image(D)
    


def handle_hive_commands(data):
    """Handles incoming commands from the hive master."""
    print data

    incomingCommand = str(data)
    command = incomingCommand[6:]

    #Check for a start command
    if command == "start":
	R.state = "active"
	R.hiveCommand = command
	StateMachine.state_start()

    #Check for a stop command
    elif command == "stop":
	R.state = "dormant"
	R.hiveCommand = command
	StateMachine.state_stop()

    #Check for a pause command
    elif command == "pause":
	R.hiveCommand = command
	StateMachine.state_wait_for_start()

    #Check for formation commands
    elif command == "line":
	R.hiveCommand = command
    elif command == "square":
	R.hiveCommand = command

    #Check for incorrect commands
    else:
	print "Invalid command."


def draw_on_image(D):
    """Displays information about the robot's current status to the Monitor window."""
    
    #Set up rectangle's position within window
    lower_left_x = 20                           
    lower_left_y = 42
    dx = 5
    dy = 5

    #Display border for rectangle
    #Border is a black rectangle under white text rectangle
    bord_upper_left = (lower_left_x-dx-3, lower_left_y-dy-20-3)
    bord_lower_right = (lower_left_x+dx+160+3, lower_left_y+dy+50+3)
    cv.Rectangle(D.image, bord_upper_left, bord_lower_right, D.black, cv.CV_FILLED)
    
    #Display white rectangle under text
    rect_upper_left = (lower_left_x-dx, lower_left_y-dy-20)
    rect_lower_right = (lower_left_x+dx+160, lower_left_y+dy+50)
    cv.Rectangle(D.image, rect_upper_left, rect_lower_right, D.white, cv.CV_FILLED)
  
    ####
    hive = "hi!"
    hiveStat = "hive"
  
    #Build Strings
    robotAssignment = ("Assignment #: %.lf"%R.assignment)
    robotConverged = ("Converged: %s"%R.converged)
    robotStatus = ("Robot Status: " + R.status)
    hiveCommand = ("Hive Command: " + hive)
    hiveStatus = ("Hive Status: " + hiveStat)
    
    # Position strings in a box so they won't overlap
    firstLineString = (lower_left_x,lower_left_y)               
    secondLineString = (lower_left_x, lower_left_y + 20)          
    thirdLineString = (lower_left_x, lower_left_y + 40)
    fourthLineString = (lower_left_x, lower_left_y + 60)
    fifthLineString = (lower_left_x, lower_left_y + 80)

    #Display strings in window
    cv.PutText(D.image, robotAssignment, firstLineString, D.font, cv.RGB(0,0,255))       
    cv.PutText(D.image, robotConverged, secondLineString, D.font, cv.RGB(0,0,255))
    cv.PutText(D.image, robotStatus, thirdLineString, D.font, cv.RGB(0,0,255))
    cv.PutText(D.image, hiveCommand, fourthLineString, D.font, cv.RGB(0,0,255))
    cv.PutText(D.image, hiveStatus, fifthLineString, D.font, cv.RGB(0,0,255))       



def check_key_press(D, key_press):
    """Handles incoming key presses."""

    if key_press == ord('q') or key_press == 27: 	#If a 'q' or ESC was pressed
	R.move(0,0)
        print "Quitting..."
        rospy.signal_shutdown( "Quit requested from keyboard" )
        
    elif key_press == ord('h'):
        print " Keyboard Command Menu"
        print " =============================="
        print " ESC/q: quit"
        print " h    : help menu"
	print " =============================="
	print " Use the arrow keys to move the robot around."
    
    #Robot keyboard driving controls
    elif key_press == 82:	#Up arrow: go forward
	R.move(80, 80)

    elif key_press == 84:	#Down arrow: go backwards
	R.move(-50, -50)
    
    elif key_press == 81:	#Left arrow: turn left
        R.move(-80, 80)

    elif key_press == 83:	#Right arrow: turn right
	R.move(80,-80)
    
    elif key_press == 32:	#Spacebar: stop
        R.move(0,0)


