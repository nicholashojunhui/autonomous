#!/usr/bin/env python
# works for actual OM ONLY!

import rospy					#import the python library for ROS
from open_manipulator_msgs.msg import JointPosition	#import JointPosition message from the open_manipulator_msgs package
from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.msg import KinematicsPose
from open_manipulator_msgs.srv import SetKinematicsPose
from geometry_msgs.msg import Pose
import math

def talker():
	rospy.init_node('OM_publisher')	#Initiate a Node called 'OM_publisher'
	set_kinematics_position = rospy.ServiceProxy('/goal_joint_space_path_to_kinematics_pose', SetKinematicsPose)
        set_gripper_position = rospy.ServiceProxy('/goal_tool_control', SetJointPosition)
	
	#while not rospy.is_shutdown():
	kinematics_pose = KinematicsPose()
	kinematics_pose.pose.position.x = 0.2867
	kinematics_pose.pose.position.y = 0.0
	kinematics_pose.pose.position.z = 0.194

	kinematics_pose.pose.orientation.w = 0.999
	kinematics_pose.pose.orientation.x = 1.765
	kinematics_pose.pose.orientation.y = 0.023
	kinematics_pose.pose.orientation.z = 0.0

	resp1 = set_kinematics_position('planning_group', 'gripper', kinematics_pose, 3)

	gripper_position = JointPosition()
	gripper_position.joint_name = ['gripper']
	gripper_position.position =  [0.01]	# -0.01 for fully close and 0.01 for fully open
	respg2 = set_gripper_position('planning_group',gripper_position, 3)

	rospy.sleep(3)
	kinematics_pose = KinematicsPose()
	kinematics_pose.pose.position.x = 0.0128
	kinematics_pose.pose.position.y = 0.2748
	kinematics_pose.pose.position.z = 0.1938

	kinematics_pose.pose.orientation.w = 0.707
	kinematics_pose.pose.orientation.x = -0.016
	kinematics_pose.pose.orientation.y = 0.016
	kinematics_pose.pose.orientation.z = 0.7058

	resp1 = set_kinematics_position('planning_group', 'gripper', kinematics_pose, 3)
	
	rospy.sleep(3)

	gripper_position = JointPosition()
	gripper_position.joint_name = ['gripper']
	gripper_position.position =  [-0.01]	# -0.01 for fully close and 0.01 for fully open
	respg2 = set_gripper_position('planning_group',gripper_position, 3)
	

if __name__== '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
