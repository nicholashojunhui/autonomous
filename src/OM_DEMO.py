#!/usr/bin/env python3

##These codes are used to move the joints of the OM that is attached to TB3.

## To use the Python MoveIt! interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
## We also import `rospy`_ and some messages that we will use:

import sys
import copy
import rospy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

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


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

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
    print("============ Reference frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print("============ End effector: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Robot Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")
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

  def arm_joints1(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
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

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def arm_joints2(self):

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


  def arm_joints3(self):

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
    joint_grip[0] = 0.005
    
    grip.go(joint_grip, wait=True)

    grip.stop()

    current_grip = self.grip.get_current_joint_values()
    return all_close(joint_grip, current_grip, 0.01)

  def gripper_close(self):

    grip = self.grip

    joint_grip = grip.get_current_joint_values()
    joint_grip[0] = -0.002
    
    grip.go(joint_grip, wait=True)

    grip.stop()

    current_grip = self.grip.get_current_joint_values()
    return all_close(joint_grip, current_grip, 0.01)

def main():
  try:
    print("============ Setting up the moveit_commander ...")
    #raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    print("============ Execute movement using 1st joint state goal ...")
    #raw_input()
    tutorial.arm_joints1()
    time.sleep(7)

    print("============ Execute gripper open ...")
    #raw_input()
    tutorial.gripper_open()
    time.sleep(3)

    print("============ Execute movement using 2nd joint state goal ...")
    #raw_input()
    tutorial.arm_joints2()
    time.sleep(7)

    print("============ Execute movement using 3rd joint state goal ...")
    #raw_input()
    tutorial.arm_joints3()
    time.sleep(7)


    print("============ Execute gripper close ...")
    tutorial.gripper_close()
    time.sleep(3)


    tutorial.arm_joints2()
    time.sleep(7)


    tutorial.arm_joints1()
    time.sleep(7)

    print("============ Python tutorial demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

