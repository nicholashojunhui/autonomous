#!/usr/bin/env python

import rospy					#import the python library for ROS
from geometry_msgs.msg import Twist		#import the twist message from the std_msgs package

def shutdown():
	
	rospy.loginfo("Stop TB3")
	pub.publish(Twist())			#default Twist() has linear.x of 0 and angular.z of 0
	rate.sleep()

def talker():
	global pub
	global rate	
	rospy.init_node('vel_publisher')	#Initiate a Node called 'vel_publisher'

	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)	#Create a Publisher object
	move = Twist()				#Create a var named move of type Twist
	rate = rospy.Rate(10)			#Set a publish rate of 10 Hz

	rospy.on_shutdown(shutdown)

	while not rospy.is_shutdown():
		move.linear.x = 0.3
		move.angular.z = 0.3
		pub.publish(move)
		rospy.loginfo("Moving in Circles")
		rate.sleep()

if __name__== '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
