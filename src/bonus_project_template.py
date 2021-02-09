#!/usr/bin/env python

import rospy	#import the python library for ROS

#import LaserScan message from the sensor_msgs.msg package
from sensor_msgs.msg import LaserScan 

#import Twist message from the geometry_msgs.msg package
from geometry_msgs.msg import Twist

#import numpy and random libraries for Markov Chain
import numpy as np
import random as rm

# Assume very first state of TB3 to be going straight
activity = "Straight"

### Defining various state functions ###
def GoStraight():
	### To fill in:



def TurnRight():
	### To fill in:



def TurnLeft():
	### To fill in:


### Constructing codes to represent autonomous exploring behaviour based on Markov Chain ###
def moveTB3():
	# The statespace
	states = ["Straight","Right","Left"]

	# Possible sequences of events
	transitionName = [["SS","SR","SL"],["RS","RR","RL"],["LS","LR","LL"]]

	# Probabilities matrix (transition matrix)
	### To fill in:




	if sum(transitionMatrix[0])+sum(transitionMatrix[1])+sum(transitionMatrix[2]) != 3:
		print("Transition matrix went wrong")
	else: print("Transition matrix is ok!!")


	# Start state of each step
	global activity
	print("Start state: " + activity)
    
	if activity == "Straight":
            change = np.random.choice(transitionName[0],replace=True,p=transitionMatrix[0])
            if change == "SS":
		GoStraight()
                pass
            elif change == "SR":
                TurnRight()
		activity = "Right"
            else:
                TurnLeft()
		activity = "Left"

        elif activity == "Right":
            ### To fill in:




                

        elif activity == "Left":
            ### To fill in:




       
	# End state of each step
	print("End state: " + activity)


### A function based on laser scan data labelled as msg ###
def callback(msg):

	### To check on relevant laser scan data ###
	print('=========================================')
	print('s1 [0]')
	print msg.ranges[0]

	print('s2 [45]')
	print msg.ranges[45]

	print('s3 [315]')
	print msg.ranges[315]

	### Constructing codes to represent collision avoidance algorithm ####
	if msg.ranges[0] >= 0.5 and msg.ranges[45] >= 0.5 and msg.ranges[315] >= 0.5:
		moveTB3()
		print("Moving based on given Markov Chain")
	### To fill in; continue codes for collision avoidance algorithm



	#Send Twist message to TB3 to make it move
	pub.publish(move)

rospy.init_node('obstacle_avoidance')			# Initiate a Node called 'obstacle_avoidance'

#Create a Subsriber object to extract the laser/scan topic message
sub = rospy.Subscriber('/scan', LaserScan, callback)

#Create a Publisher object to move the TB3
pub = rospy.Publisher('/cmd_vel', Twist)
move = Twist()

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
