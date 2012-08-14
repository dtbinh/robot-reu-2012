#!/usr/bin/env python
import roslib; roslib.load_manifest('robot_convoy')
import rospy
import socket
import httplib
import time
import irobot_mudd
import cv_bridge
import cv
import sensor_msgs.msg as sm
from std_msgs.msg import String
from irobot_mudd.srv import *
from irobot_mudd.msg import *
import Navigator
import Robot
import ImageProcessing
import RangeProcessing
import StateMachine


################################################################################
###
### Antonella Wilby and Vivian Wehner
###
### This module handles incoming data from the Robot's sensorPacket publisher,
### the Kinect's RGB image stream, the Kinect's range image stream, incoming key
### presses, and mouse input to all windows.
###
################################################################################



#Get instances of Data and Robot from Navigator
D = Navigator.get_data_instance()
R = Navigator.get_robot_instance()



################## BEGIN DATA HANDLING FUNCTIONS ###################

def handle_sensor_data(data):
    """Handle_sensor_data is called every time the robot gets a new sensorPacket."""
    
    #print dir( data )
    D.data = data

    #Check for a bump
    if data.bumpRight or data.bumpLeft:
        print "Bumped!"


    #Check if play button was pressed	
    if data.play:
	print "Play button pressed!"
	StateMachine.state_stop()
	rospy.signal_shutdown("play button pressed")
	

    
def handle_image_data(data):
    """Handles incoming RGB image data from the Kinect."""
    
    #Get the incoming RGB image from the Kinect
    D.image = D.bridge.imgmsg_to_cv(data, "bgr8")

    if D.created_images == False:
        #Initialize the additional images we need for processing
        ImageProcessing.initialize(D)
        D.created_images = True

    # Recalculate threshold image
    ImageProcessing.threshold_image(D)

    # Recalculate blob in main image
    ImageProcessing.find_biggest_region(D)

    # Check on the display of dragged section
    ImageProcessing.mouse_section(D)

    #Display target circle
    #ImageProcessing.target_coord(D)
    
    #Display info box on image
    ImageProcessing.draw_on_image(D)
    
    #Handle incoming key presses
    key_press = cv.WaitKey(5) & 255
    if key_press != 255:			#Handle only if it's a real key
        check_key_press(D, key_press)		#(255 = "no key pressed")

    #Update the displays:
    #Show main image in the image window
    cv.ShowImage('Image', D.image)

    #Show threshold image in the threshold window
    currentThreshold = getattr(D, D.current_threshold)
    cv.ShowImage('Threshold', currentThreshold)


def handle_range_image(data):
    """Handles the data each time the kinect sends a range image."""
    
    #Get the incoming depth image from the kinect
    D.range = D.bridge.imgmsg_to_cv(data, "32FC1")
    
    if D.range_image == False:
	#Initialize the range image from kinect sensor
	RangeProcessing.initialize(D)
	D.range_image = True

    #Calculate horizontal and vertical angles and display in window
    D.xAngle = RangeProcessing.calculate_angles(D, D.p1, D.p2, "horizontal")
    D.yAngle = RangeProcessing.calculate_angles(D, D.p3, D.p4, "vertical")
    RangeProcessing.draw_on_image(D)

    #Handle incoming key presses
    key_press = cv.WaitKey(5) & 255
    if key_press != 255:			#Handle only if it's a real key
        check_key_press(D, key_press)		#(255 = "no key pressed")
    
    #Display the image in the Range Window
    cv.ShowImage('Range', D.range)


def handle_keyboard_data(data):
    """Handles all input from the keyboard."""
    pass

def handle_mouse_data(data):
    """Handles all input from the mouse."""
    pass




def mouseImage(event, x, y, flags, param):
    """Handles incoming mouse input to the Image window."""
    
    if event==cv.CV_EVENT_LBUTTONDOWN:  #Clicked the left button
        print "x, y are", x, y
        (b,g,r) = D.image[y,x]
        print "r,g,b is", int(r), int(g), int(b)
        (h,s,v) = D.hsv[y,x]
        print "h,s,v is", int(h), int(s), int(v)
        D.down_coord = (x,y)
        D.mouse_down = True
        
    elif event==cv.CV_EVENT_LBUTTONUP:  #Let go of the left button
        print "x, y are", x, y
        (b,g,r) = D.image[y,x]
        print "r,g,b is", int(r), int(g), int(b)
        (h,s,v)  = D.hsv[y,x]
        print "h,s,v is", int(h), int(s), int(v)
        D.up_coord = (x,y)
        D.mouse_down = False

        if D.mode == "clear":
            D.sections = []
        else:      #Start, add, or subtract -- put lower coordinates first
            x0, y0, x1, y1 = D.down_coord[0], D.down_coord[1], D.up_coord[0], D.up_coord[1]

            if x0 > x1:
                x0, x1 = x1, x0
            if y0 > y1:
                y0, y1 = y1, y0
            
            if D.mode == "start":
                D.sections = []
            mode_dict = {"start":'a', "add":'a', "subtract":'s'}
            D.sections.append([mode_dict[D.mode], (x0, y0), (x1, y1)])
            ImageProcessing.process_section(D)


    elif event == cv.CV_EVENT_RBUTTONDOWN:                      #Right click
        D.target_coord = (x, y)
        ImageProcessing.target_coord(D)


    elif D.mouse_down and event==cv.CV_EVENT_MOUSEMOVE:      #Mouse just moved
        D.up_coord = (x,y)


def mouseRange(event, x, y, flags, param):
    """Handles incoming mouse input to the Range window."""
    
    #If the left button was clicked
    if event==cv.CV_EVENT_LBUTTONDOWN:
        print "x, y are", x, y
        pixel_val= D.image[y,x]
        print "the pixel's depth value is", pixel_val
        if D.mode == "setLeft":
            D.dot1 = (x,y)
            D.mode = D.lastmode
        elif D.mode == "setRight":
            D.dot2 = (x,y)
            D.mode = D.lastmode
        elif D.mode == "setTop":
            D.dot3 = (x,y)
            D.mode = D.lastmode
        elif D.mode == "setDown":
            D.dot4 = (x,y)
            D.mode = D.lastmode


################### END DATA HANDLING FUNCTIONS ####################


######## TEMPORARY CALLBACK FUNCTIONS UNTIL WE GET TOPIC PUBLISHING/SUBSCRIBING WORKING



def check_key_press(D, key_press):
    """Handles incoming key presses."""
    
    D.last_key_pressed = key_press

    if key_press == ord('q') or key_press == 27: # if a 'q' or ESC was pressed
	R.move(0,0)
        print "quitting"
        rospy.signal_shutdown( "Quit requested from keyboard" )
        
    elif key_press == ord('h'):
        print " Keyboard Command Menu"
        print " =============================="
        print " ESC/q: quit"
        print " h    : help menu"
        print " s    : save thresholds to file"
        print " l    : load thresholds from file"
        print " c    : mousedrags will no longer set thresholds, kept values will be cleared"
        print " a    : mousedrag will assign thresholds to area within drag, \n" + \
              "        resets on new click or drag"
        print " r    : mousedrags will remove the area under consideration, \n" + \
              "        must have set an area in 'a' mode first"
        print " m    : mousedrags will add the area under consideration, \n" + \
              "        must have set an area in 'a' mode first"
        print " t    : show total threshold image in threshold window"
        print " A    : activate robot for moving, press A again to deactivate "
        print " 1    : begin state machine as leader"
	print " 2    : begin state machine as follower"



    #Save thresholds to file
    elif key_press == ord('s'):
        fileName = raw_input('Please enter the name of a color: ')
        fileName += "_thresholds.txt"
        writeFile = open(fileName, "w")         #open file for writing
        print >> writeFile, D.thresholds
        writeFile.close()


    #Load thresholds from file    
    elif key_press == ord('l'):
        whichFile = raw_input('Please enter the name of a color: ')
        whichFile += "_thresholds.txt"
        readFile = open(whichFile, "r")        #open file for reading
        data = readFile.read()
        D.thresholds = eval(data)
        readFile.close()
	
	D.loaded_thresholds = True

        #Reset threshold sliders
        #for thresh in ['red', 'blue', 'green', 'hue', 'sat', 'val']:
        #    cv.SetTrackbarPos('low_' + thresh, 'Sliders', D.thresholds['low_'+thresh])
        #   cv.SetTrackbarPos('high_' + thresh, 'Sliders', D.thresholds['high_'+thresh])

    # Start picking up thresholded images
    elif key_press == ord('a'):
        D.mode = "start"


    # Clear all loaded sections
    elif key_press == ord('c'):
        D.mode = "clear"
        D.sections = []


    # Remove areas from thresholding
    elif key_press == ord('r'):
        if len(D.sections) > 0:
            D.mode = "subtract"
        else:
            print "Cannot switch modes, need a starting area first. Press 'i' " + \
                "to select a starting area."

    # Add areas for thresholding
    elif key_press == ord('m'):
        if len(D.sections) > 0:
            D.mode = "add"
        else:
            print "Cannot switch modes, need a starting area first. Press 'i' " + \
                "to select a starting area."

    elif key_press == ord('t'):
        D.current_threshold = D.threshed_image

    # Activate the robot for moving
    elif key_press == ord('A'):
        StateMachine.activate()

    # Activate robot as leader, following a white line
    elif key_press == ord('1'):
	print "Setting mode to \"leader\"."
	StateMachine.state_start("leader")
    
    # Activate robot as follower, following the defined target
    elif key_press == ord('2'):
	print "Setting mode to \"follower\"."
	StateMachine.state_start("follower")
    
    # Client activation, can send and recieve
    elif key_press == ord ('w'):
		
	serverHost = 'localhost'
	serverHost = '192.168.1.101'
	#serverHost = '134.173.195.75'
	#serverHost = "134.173.195.75"
	serverHost = "134.173.43.14"
    
	serverPort = 2012
	
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)     #Create a TCP socket
	s.connect((serverHost, serverPort))  #Connect to server on the port
	
	# Figure out what to send
	message = ""

	if True:
	    s.send("hi") #send the data 
	    data = s.recv(1024) #receive up to 1K bytes 
	    print "I received the message:", data
	    
    
	s.close()
	print 'Good bye!'
    
    

    #Robot keyboard driving controls
    elif key_press == 82:		#up arrow: drive forward
	R.move(250, 250)

    elif key_press == 84:		#down arrow: drive backward
	R.move(-250, -250)
    
    elif key_press == 81:		#left arrow: turn left
        R.move(-250, 250)

    elif key_press == 83:		#right arrow: turn right
	R.move(250,-250)
    
    elif key_press == 32:		#spacebar: stop
        R.move(0,0)
            


#########################################################################
##################     Connectivity functions	#########################

#
# example of status checking
#
def get_status():
    conn = httplib.HTTPConnection("www.cs.hmc.edu", 80, timeout=0.5)
    conn.request("GET","/~vwehner/robotconvoy/index.py?status=1")
    try:
	res = conn.getresponse()
	result_string = res.read().strip() # get rid of extra space
	#print "result_string is", result_string
	if type(eval(result_string)) == dict:
	    D.robots = eval( result_string )
	else:
	    print "Not a dictionary!"
    except:
	print "Oops Timeout in get status"

#
# example of status setting
#
def set_status(robot_name, status_to_be_set):
    conn = httplib.HTTPConnection("www.cs.hmc.edu", 80, timeout=0.5)
    #print "robot ", robot_name
    # Make sure the local version fits the websites
    D.robots[robot_name] = status_to_be_set	
    request_string = "/~vwehner/robotconvoy/index.py?" + robot_name + "=" + str(status_to_be_set)
    # set request
    conn.request("GET",request_string)


def set_all(status):
    for key in D.robots:
	set_status(key, status)


# Like set_all_but_stopped, except it adds on that you are who everyone is waiting on
def make_others_wait():
    for key in D.robots:
	if key != R.name:
	    if D.robots[key] != "stop": 
		set_status(key, "wait")
	    else: # Don't reset stopped robots
		print "can't make a stopped robot wait"
		pass
	else:
	    set_status(key, "stop") # You are who everyone is waiting on


def start_up_who_can():
    # Start up everyone who was waiting, but if someone else is stopped,
    # we and everyone else needs to wait
    get_status()
    for key in D.robots:
	if D.robots[key] == "stop": # If someone else is stopped
	    #print "we found a stopped one"
	    for key in D.robots:
		if D.robots[key] == "stop":
		    pass
		else:
		    set_status(key, "wait")
	    break
        else:
	    #print "yep not stopped"
	    set_status(key, "go")   # Otherwise we can go!


def hello():
    print "hello world"
    
    

