#! /usr/bin/env python

##
# \file print_info.py
# \brief Node that prints information about the position and velocity of a robot
# \author Aurora Bottino
# \date 24/01/2023
# \details
#
# This node subscribes to the "/pos_vel" topic and prints information about the robot's
# position and velocity. It also computes the distance from the desired position and the
# average speed of the robot. The desired position is obtained from the parameters "des_pos_x"
# and "des_pos_y", and the publish frequency is obtained from the parameter "frequency".
#

import rospy
import math
import time
from assignment_2_2022.msg import Position_velocity

# Printed info frequency 
freq = 1.0

# Last time the info was printed
printed = 0


##
# \brief Callback function for the "/pos_vel" subscriber
# This function is called every time a message is received from the "/pos_vel" topic.
# It computes the distance from the desired position and the average speed of the robot
# and prints the information if the publish frequency has been reached.
# \param msg The message received from the "/pos_vel" topic
#


def posvel(msg):
	
   global freq, printed
	
	# Compute time period in milliseconds
   period = (1.0/freq) * 1000
	
	# Get current time in milliseconds
   curr_time = time.time() * 1000
	
	
   if curr_time - printed > period:
		
		# Get the desired position
        des_x = rospy.get_param("des_pos_x")
        des_y = rospy.get_param("des_pos_y")
		
		# Get the actual position
        x = msg.x
        y = msg.y
		
		# Compute the distance
        dist = math.dist([des_x, des_y], [x, y])
		
		# Compute the average speed
        average_speed = math.sqrt(msg.v_x**2 + msg.v_y**2)
		
		# print info
        print("Missing distance from desired position: ", dist)
        print("Average speed: ", average_speed,"\n")
		
		# Update printed
        printed = curr_time
	

##
# \brief Main function
# This function initializes the node and subscribes to the "/pos_vel" topic.
# It also obtains the desired position and publish frequency from the parameters
# and calls the "posvel" callback function every time a message is received.
#


def main():
		
	# Global variable
    global freq
	
	# Initialize the node
    rospy.init_node('print_info')
	
	# Get the publish frequency parameter
    freq = rospy.get_param("frequency")
	
	# subscriber get from "pos_vel" a parameter (Position_velocity message)
    sub_pos = rospy.Subscriber("/pos_vel", Position_velocity, posvel)
	
	# Wait
    rospy.spin()
	
if __name__ == "__main__":
	main()	
	# Get the publish frequency parameter
    freq = rospy.get_param("frequency")
	
	# subscriber get from "pos_vel" a parameter (Position_velocity message)
    sub_pos = rospy.Subscriber("/pos_vel", Position_velocity, posvel)
	
	# Wait
    rospy.spin()
	
if __name__ == "__main__":
	main()	
