#!/usr/bin/env python

##These codes are used to move the joints of the OM that is attached to TB3 for the pick and place actions. Codes to move the base (i.e. the TB3) are integrated here too.

import os,sys
import actionlib
import copy
import rospy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


############################ Defining Navigation Parameters ############################ 

#path planning sequences (loop phase)
#define waypoints here; you may edit this to your preferences
waypoints = [
    [ (-0.5, 0.0, 0.0),
      (0.0, 0.0, 0.0, 1.0)]
]

#goal_pose default parameters; do not edit this
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


############################ Defining Manipulation Parameters ############################ 

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonInteface(object):
  """MoveGroupPythonInteface"""
  def __init__(self):
    super(MoveGroupPythonInteface, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_move_place_Node', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the OM
    ## arm so we set ``group_name = arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the OM:
    group_name = "arm"
    group_name2 = "gripper"
    group = moveit_commander.MoveGroupCommander(group_name)
    grip = moveit_commander.MoveGroupCommander(group_name2)

    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.grip = grip
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

############################ Defining Joint Parameters ############################ 

  def Front(self):
    group = self.group

    ## Planning to a Joint Goal
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    
    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def Left(self):

    group = self.group

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 1.571
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    
    group.go(joint_goal, wait=True)

    group.stop()

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def Down(self):

    group = self.group

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 1.571
    joint_goal[1] = 0.526
    joint_goal[2] = 0
    joint_goal[3] = 0

    group.go(joint_goal, wait=True)

    group.stop()

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def gripper_open(self):

    grip = self.grip

    joint_grip = grip.get_current_joint_values()
    joint_grip[0] = 0.01
    
    grip.go(joint_grip, wait=True)

    grip.stop()

    current_grip = self.grip.get_current_joint_values()
    return all_close(joint_grip, current_grip, 0.01)

  def gripper_close(self):

    grip = self.grip

    joint_grip = grip.get_current_joint_values()
    joint_grip[0] = -0.01
    
    grip.go(joint_grip, wait=True)

    grip.stop()

    current_grip = self.grip.get_current_joint_values()
    return all_close(joint_grip, current_grip, 0.01)

def pick():
  try:
    # Setting up the moveit_commander ...
    move_OM = MoveGroupPythonInteface()

    # Execute movement using 1st joint state goal; move to left ...
    move_OM.Left()
    time.sleep(7)

    # Execute gripper open ...
    move_OM.gripper_open()
    time.sleep(3)

    # Execute movement using 2nd joint state goal; move down ...
    move_OM.Down()
    time.sleep(7)

    #Execute gripper close ...
    move_OM.gripper_close()
    time.sleep(3)

    # Execute movement using 3rd joint state goal; move up ...
    move_OM.Left()
    time.sleep(7)

    # Execute movement using 4th joint state goal; face front ...
    move_OM.Front()
    time.sleep(7)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


def place():
  try:
    # Setting up the moveit_commander ...
    move_OM = MoveGroupPythonInteface()

    # Execute movement using 1st joint state goal; move to left ...
    move_OM.Left()
    time.sleep(7)

    # Execute movement using 2nd joint state goal; move down ...
    move_OM.Down()
    time.sleep(7)

    # Execute gripper open ...
    move_OM.gripper_open()
    time.sleep(3)

    # Execute movement using 3rd joint state goal; move up ...
    move_OM.Left()
    time.sleep(7)

    # Execute movement using 4th joint state goal; face front ...
    move_OM.Front()
    time.sleep(7)

    #Execute gripper close ...
    move_OM.gripper_close()
    time.sleep(3)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

######## Configuring Sequence of Function Execution in Main ######## 

if __name__ == '__main__':

  #allow time for launch setup to be completed
  time.sleep(5)

  #use OM to pick item from ground
  pick()

  #Move TB3 to desired position/orientation
  client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

  # wait for action server to be ready
  client.wait_for_server()

  for pose in waypoints:
  	goal = goal_pose(pose)
        print("Going for goal: ", goal)
        client.send_goal(goal)
        client.wait_for_result()

  #use OM to place item on ground
  place()




