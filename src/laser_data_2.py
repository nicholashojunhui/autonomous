#!/usr/bin/env python3

import rospy					#import the python library for ROS
from sensor_msgs.msg import LaserScan		#import the laserscan message from the std_msgs package

def callback(msg):				#define a function called 'callback' that receives a parameter named 'msg'

#LDS-02 has a publishing data size of 234 instead of the usual 360. Hence you must adjust the angle according to your LDS type

	print('=========================================')
	print('Front')				#value for front-direction laser beam
	print(msg.ranges[0])			#print the distance to any obstacle in front of the robot. The sensor returns a vector of 359 values, with 0 being in front of the TB3

	print('Right')
	print(msg.ranges[58])

	print('Back')
	print(msg.ranges[117])

	print('Left')
	print(msg.ranges[175])

rospy.init_node('laser_data')			# Initiate a Node called 'laser_data'
sub = rospy.Subscriber('scan', LaserScan, callback) #Create a Subsriber to the laser/scan topic

rospy.spin()
