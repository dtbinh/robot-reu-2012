#!/usr/bin/env python
import roslib; roslib.load_manifest('formation_control')
import rospy
import irobot_mudd
from irobot_mudd.srv import *
from irobot_mudd.msg import *



def handle_sensor_data(data):
    """Handles all incoming sensorPackets from the robot."""

    #Store incoming data in the Data object
    #D.data = data

    #Check for a bump
    if data.bumpRight or data.bumpLeft:
        print "Bumped!"


    #Check if play button was pressed   
    if data.play:
        print "Play button pressed!"
        StateMachine.state_stop()
        rospy.signal_shutdown("play button pressed")


    #Check key presses
    key_press = cv.WaitKey(5) & 255
    if key_press != 255:
        check_key_press(D, key_press)



def check_key_press(D, key_press):
    """Handles incoming key presses."""

    if key_press == ord('q') or key_press == 27:        #If a 'q' or ESC was pressed
        print "Qutting..."
        rospy.signal_shutdown( "Quit requested from keyboard" )

    elif key_press == ord('h'):
        print " Keyboard Command Menu"
        print " =============================="
        print " ESC/q: quit"
        print " h    : help menu"
	print " =============================="
	print "Use the arrow keys to drive the robot."

    #Robot keyboard driving controls
    elif key_press == 82:       #Up arrow: go forward
        R.move(100, 100)

    elif key_press == 84:       #Down arrow: go backwards
        R.move(-100, -100)

    elif key_press == 81:       #Left arrow: turn left
        R.move(-100, 100)

    elif key_press == 83:       #Right arrow: turn right
        R.move(100,-100)

    elif key_press == 32:       #Spacebar: stop
        R.move(0,0)



