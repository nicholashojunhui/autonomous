#!/usr/bin/env python

import rospy
import os,sys
import actionlib
import math

# move_base is the package that takes goals for navigation
# there are different implemenetations with a common interface
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# additional package to import to convert euler to quaternion
from tf.transformations import quaternion_from_euler

rospy.init_node('path_planning2_Node')

# You need to know the coordinates that the map you are working
# in is. I estimated these numbers using the turtlebot3_world
# map from Turtlebot3. The center of the map is (0.0, 0.0, 0.0); each grid is 1m.

# Quaternions are a different mathematical represetnation 
# for "euler angles", roll, pitch and yaw (RPY).
# below shows how to convert Euler angles (i.e. RPY) to Quaternions
# Note that for the TB3, only the Yaw component is needed
# Change theta to change the orientation of the TB3. E.g. 0 degrees is facing front, 90 degrees is facing left and -90 degress is facing right

#theta in degrees
theta1 = 90
yaw_radians1 = theta1 * (math.pi/180)

theta2 = -90
yaw_radians2 = theta2 * (math.pi/180)

# in order RPY
q1 = quaternion_from_euler(0, 0, yaw_radians1)
q2 = quaternion_from_euler(0, 0, yaw_radians2)

#The first waypoint array is x,y,z location. 
#The second one is a "quaternion" defining an orientation. 

#path planning sequences (loop phase)
waypoints = [
    [ (-0.5, 0.0, 0.0),			#centre of map
      (q1[0], q1[1], q1[2], q1[3])],	
   [ (0.0, 2.0, 0.0),			#left of map
      (q2[0], q2[1], q2[2], q2[3])]
]

def goal_pose(pose):
	goal_pose = MoveBaseGoal()
	goal_pose.target_pose.header.frame_id = 'map'
	goal_pose.target_pose.pose.position.x = pose[0][0]
	goal_pose.target_pose.pose.position.y = pose[0][1]
	goal_pose.target_pose.pose.position.z = pose[0][2]
	goal_pose.target_pose.pose.orientation.x = pose[1][0]
	goal_pose.target_pose.pose.orientation.y = pose[1][1]
	goal_pose.target_pose.pose.orientation.z = pose[1][2]
	goal_pose.target_pose.pose.orientation.w = pose[1][3]
	return goal_pose

# Main program starts here
if __name__ == '__main__':
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    	client.wait_for_server()

    	# Loop until ^c; delete this line if you want the TB3 to stop at the last waypoint
    	while not rospy.is_shutdown():

        	# repeat the waypoints over and over again
        	for pose in waypoints:
            		goal = goal_pose(pose)
            		print("Going for goal: ", goal)
            		client.send_goal(goal)
            		client.wait_for_result()

