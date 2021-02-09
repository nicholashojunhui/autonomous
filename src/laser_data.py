#!/usr/bin/env python

import rospy					#import the python library for ROS
from sensor_msgs.msg import LaserScan		#import the laserscan message from the std_msgs package

def callback(msg):				#define a function called 'callback' that receives a parameter named 'msg'
	print('=========================================')
	print('s1 [0]')				#value for front-direction laser beam
	print msg.ranges[0]			#print the distance to any obstacle in front of the robot. The sensor returns a vector of 359 values, with 0 being in front of the TB3

	print('s2 [90]')
	print msg.ranges[90]

	print('s3 [180]')
	print msg.ranges[180]

	print('s4 [270]')
	print msg.ranges[270]

	print('s5 [359]')
	print msg.ranges[359]

rospy.init_node('laser_data')			# Initiate a Node called 'laser_data'
sub = rospy.Subscriber('scan', LaserScan, callback) #Create a Subsriber to the laser/scan topic

rospy.spin()
