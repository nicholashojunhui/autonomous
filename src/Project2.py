#!/usr/bin/env python3
import rospy					#import the python library for ROS
from open_manipulator_msgs.msg import JointPosition	#import JointPosition message from the open_manipulator_msgs package
from open_manipulator_msgs.srv import SetJointPosition
import math

def talker():
	rospy.init_node('OM_publisher')	#Initiate a Node called 'OM_publisher'
	set_joint_position = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)
	set_gripper_position = rospy.ServiceProxy('/goal_tool_control', SetJointPosition)
	
	# Loop until ^c; comment this line if you want the arm to stop at the last joint set
	#while not rospy.is_shutdown():
	joint_position = JointPosition()
	joint_position.joint_name = ['joint1','joint2','joint3','joint4']

	#Step 1
	joint_position.position =  [0, 0, 0, 0]		# in radians
	resp1 = set_joint_position('planning_group',joint_position, 3)
	
	#rospy.sleep(3)
	gripper_position = JointPosition()
	gripper_position.joint_name = ['gripper']
	gripper_position.position =  [0.01]	# -0.01 for fully close and 0.01 for fully open
	respg2 = set_gripper_position('planning_group',gripper_position, 3)

	rospy.sleep(3)

	#Step 2
	joint_position.position =  [1.571, 0, 0, 0]		# in radians
	resp1 = set_joint_position('planning_group',joint_position, 3)

	rospy.sleep(3)

	#Step 3a
	joint_position.position =  [1.571, 0.526, 0, 0]		# in radians
	resp1 = set_joint_position('planning_group',joint_position, 3)

	rospy.sleep(3)		

	#Step 3b
	gripper_position.position =  [-0.01]	# -0.01 for fully close and 0.01 for fully open
	respg2 = set_gripper_position('planning_group',gripper_position, 3)

	rospy.sleep(3)

	#Step 4
	joint_position.position =  [1.571, 0, 0, 0]		# in radians
	resp1 = set_joint_position('planning_group',joint_position, 3)

	rospy.sleep(3)

	#Step 5
	joint_position.position =  [0, 0, 0, 0]		# in radians
	resp1 = set_joint_position('planning_group',joint_position, 3)

	rospy.sleep(3)

	#Step 6
	joint_position.position =  [-1.571, 0, 0, 0]		# in radians
	resp1 = set_joint_position('planning_group',joint_position, 3)

	rospy.sleep(3)

	#Step 7a
	joint_position.position =  [-1.571, 0.526, 0, 0]		# in radians
	resp1 = set_joint_position('planning_group',joint_position, 3)

	rospy.sleep(3)		

	#Step 7b
	gripper_position.position =  [0.01]	# -0.01 for fully close and 0.01 for fully open
	respg2 = set_gripper_position('planning_group',gripper_position, 3)

	rospy.sleep(3)

	#bring arm up
	joint_position.position =  [-1.571, 0, 0, 0]		# in radians
	resp1 = set_joint_position('planning_group',joint_position, 3)

	rospy.sleep(3)

	#initial pose
	joint_position.position =  [0, 0, 0, 0]		# in radians
	resp1 = set_joint_position('planning_group',joint_position, 3)


if __name__== '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
