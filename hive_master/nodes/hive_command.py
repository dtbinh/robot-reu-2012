#!/usr/bin/env python
import roslib; roslib.load_manifest('hive_master')
import rospy
from std_msgs.msg import String


def wait_for_input():
    """Waits for user to input a command."""
    
    prompt = '>>: '
    input = raw_input(prompt)

    #All valid input the command line can take. 
    #Program commands listed at the beginning
    #Robot commands listed at the end
    valid_input = ["help", "quit", "start", "stop", "pause", "line", "square"]

    robot_input = valid_input[2:]
    
    #Validate user input
    while not input in valid_input:
	print "That input is not valid.  Type \'help\' for a list of valid commands."
	input = raw_input(prompt)

    if input in robot_input:
	return input
    elif input == "help":
	print " Hive Master Commands"
	print " ========================================="
	print " start : sends start command to all robots"
	print " stop  : sends stop command to all robots"
	print " pause : sends pause command to all robots"
	print " line  : sends command to form line"
	print " square: sends command to form square"
	print " quit  : quits hive command program"
	wait_for_input()
    elif input == "quit":
	rospy.signal_shutdown("User has quit the program.")
	print "Quitting..."


def main():
    """Initializes node and sets up publisher for commands from the hive master."""

    rospy.init_node('hive_commands')
    commands = rospy.Publisher('hive_commands', String)
    
    print "Command interface started. Ready to receive commands."
    print "====================================================="

    while not rospy.is_shutdown():
	input = wait_for_input()
        rospy.loginfo(input)
	commands.publish(String(input))
	rospy.sleep(0.3)


if __name__ == "__main__":
    main()
