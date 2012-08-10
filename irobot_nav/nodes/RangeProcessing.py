#!/usr/bin/env python
import roslib; roslib.load_manifest('irobot_nav')
import rospy
import irobot_mudd
import cv_bridge
import cv
import sensor_msgs.msg as sm
from std_msgs.msg import String
import numpy
import math, cmath



######################### INITIALIZATION FUNCTIONS #########################

def initialize(D):
    """Sets up everything needed for processing range images."""
    
    #Set up stuff needed to process range images
    D.init = True
    D.changed = False
    D.depth1, D.depth2, D.depth3, D.depth4 = 0, 0, 0, 0
    D.xAngle, D.yAngle = 0, 0
    


############### RANGE PROCESSING FUNCTIONS #########################

def draw_on_image(D):
    """Draws circles on image at points p1 and p2."""

    # Draw circles on the points you choose
    cv.Circle(D.range, D.p1, 12, D.white, thickness=1, lineType=8, shift=0)
    cv.Circle(D.range, D.p1, 10, D.black, thickness=1, lineType=8, shift=0)
    cv.Circle(D.range, D.p1, 8, D.white, thickness=1, lineType=8, shift=0)

    cv.Circle(D.range, D.p2, 12, D.white, thickness=1, lineType=8, shift=0)
    cv.Circle(D.range, D.p2, 10, D.black, thickness=1, lineType=8, shift=0)
    cv.Circle(D.range, D.p2, 8, D.white, thickness=1, lineType=8, shift=0)

    cv.Circle(D.range, D.p3, 12, D.white, thickness=1, lineType=8, shift=0)
    cv.Circle(D.range, D.p3, 10, D.black, thickness=1, lineType=8, shift=0)
    cv.Circle(D.range, D.p3, 8, D.white, thickness=1, lineType=8, shift=0)

    cv.Circle(D.range, D.p4, 12, D.white, thickness=1, lineType=8, shift=0)
    cv.Circle(D.range, D.p4, 10, D.black, thickness=1, lineType=8, shift=0)
    cv.Circle(D.range, D.p4, 8, D.white, thickness=1, lineType=8, shift=0)

    #Draw box for text display
    dx, dy = 5, 5                           #Set up rectangle's position within window
    lower_left_x, lower_left_y = 20,42
    
    #Display black border around box
    bord_upper_left = (lower_left_x-dx-3, lower_left_y-dy-20-3)
    bord_lower_right = (lower_left_x+dx+120+3, lower_left_y+dy+50+3)
    cv.Rectangle(D.range, bord_upper_left, bord_lower_right, D.black, cv.CV_FILLED)
    
    #Display white box 
    rect_upper_left = (lower_left_x-dx, lower_left_y-dy-20)
    rect_lower_right = (lower_left_x+dx+120, lower_left_y+dy+50)
    cv.Rectangle(D.range, rect_upper_left, rect_lower_right,
                 D.white, cv.CV_FILLED)
    
    #Calculate average depth
    avgDepth = (D.depth1 + D.depth2 + D.depth3 + D.depth4) / 4.0
    
    #Print text in info box
    value_string = ("Avg. Depth: %.1f" % avgDepth) # formatted printing
    string_lower_left = (lower_left_x,lower_left_y)
    cv.PutText(D.range, value_string, string_lower_left, D.font, D.black)

    value_string3 = ("Y Angle %.1f" % D.yAngle) # formatted printing
    string_lower_left3 = (lower_left_x,lower_left_y + 20)
    cv.PutText(D.range, value_string3, string_lower_left3, D.font, D.black)

    value_string4 = ("X Angle %.1f" % D.xAngle) # formatted printing
    string_lower_left4 = (lower_left_x,lower_left_y + 40)
    cv.PutText(D.range, value_string4, string_lower_left4, D.font, D.black)


def calcDist((x0,y0),(x1,y1)):
    """Calculates the distance between two points."""
    
    deltaX = x0 - x1
    deltaY = y0 - y1
    return math.sqrt(deltaX**2 + deltaY**2)


def convert(x,y):
    """Converts x and y coordinates into the world coordinate system."""
    
    return (x - 320, y - 240)


def calculate_angles(D, point1, point2, orientation):
    """Looks at two points in the depth image and gets the angles in between them."""
    
    # Convert pixel dimensions to meters
    degPerPix = 0
    p1_x, p1_y, p2_x, p2_y = point1[0], point1[1], point2[0], point2[1]
    
    # Convert so points are on a -320,320 and -240,240 scale
    convertedPair1, convertedPair2 = convert(p1_x,p1_y), convert(p2_x,p2_y)
    
    if orientation == "vertical":
        D.depth3 = D.range[p1_y,p1_x]
        D.depth4 = D.range[p2_y, p2_x]
        degPerPix = 43.0/480.0
        
        # Changing point on plane to degree from origin
        p1Angle, p2Angle = convertedPair1[1]* degPerPix, \
                         convertedPair2[1] * degPerPix
    else:
        D.depth1 = D.range[p1_y,p1_x]
        D.depth2 = D.range[p2_y, p2_x]
        degPerPix = 57.0/640.0                  #degree/pixel
        
        # Changing point on plane to degree from origin
        p1Angle, p2Angle = convertedPair1[0]* degPerPix, \
                         convertedPair2[0] * degPerPix

    # Figuring out real world position of points on kinect plane
    length1, length2 = math.tan(math.radians(p1Angle)),\
                         math.tan(math.radians(p2Angle))

    distBetween = length2 - length1


    # Final Angle calculations
    if math.isnan(D.range[p2_y, p2_x]) or math.isnan(D.range[p1_y, p1_x]):
        return 0.00        #return nan
    else:
        angleBetween = math.atan2((D.range[p1_y, p1_x]- D.range[p2_y, p2_x]),\
                                  distBetween)
        return math.degrees(angleBetween)
    
####################### END RANGE PROCESSING FUNCTIONS #######################


