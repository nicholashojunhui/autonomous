#!/usr/bin/env python3

import os,sys
import actionlib
import copy
import rospy
import time

#import packages for manipulation purposes
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

#import packages for follow line purposes
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot_turtlebot3 import MoveTB3

#from math import pi
#from std_msgs.msg import String

############################ Defining Line Follower Parameters ############################ 

cx = 0
cy = 0

class LineFollower(object):

    def __init__(self):
    
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/raspicam_node/image",Image,self.camera_callback)
	#self.image_sub = rospy.Subscriber("/raspicam_node/image_raw",Image,self.camera_callback)
        self.moveTB3_object = MoveTB3()

    def camera_callback(self,data):
        
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        print('Working')
            
        # We get image dimensions and crop the parts of the image we dont need
        # Bare in mind that because its image matrix first value is start and second value is down limit.
        # Select the limits so that it gets the line not too close, not too far and the minimum portion possible
        # To make process faster.
        height, width, channels = cv_image.shape
	#print(height)
        descentre = 160
        rows_to_watch = 20
        crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]
        
        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        
        # Define the Yellow Colour in HSV
        #[[[ 30 196 235]]]
        """
        To know which color to track in HSV, Put in BGR. Use ColorZilla to get the color registered by the camera
        >>> yellow = np.uint8([[[B,G,R]]])
        >>> hsv_yellow = cv2.cvtColor(yellow,cv2.COLOR_BGR2HSV)
        >>> print( hsv_yellow )
        [[[ 60 255 255]]]
        """
        lower_yellow = np.array([20,100,50])
        upper_yellow = np.array([50,255,255])

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)

	global cx
	global cy

        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cx, cy = height/2, width/2
        
	print('cx',cx) 
	print('cy',cy)         

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(crop_img,crop_img, mask= mask)
        
        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(res,(int(cx), int(cy)), 10,(0,0,255),-1)

        cv2.imshow("Original", cv_image)
        #cv2.imshow("HSV", hsv)
        #cv2.imshow("MASK", mask)
        cv2.imshow("RES", res)
        
        cv2.waitKey(1)

	
        try:
            a
        except:
            error_x = 0
	    error_1 = 0
	    error_2 = 0
	       
	error_2 = error_1
	error_1 = error_x 	
        error_x = cx - width / 2;	
	a = error_x
        twist_object = Twist();
        twist_object.linear.x = 0.1;
        kp = -1/500
 	kd = 1
	ki = 0
	k1 =kp + ki + kd
	k2 = -kp -2*kd
	k3 = kd
	print('0',error_x,'1',error_1,'2',error_2)
        twist_object.angular.z = -error_x/600 -error_1/1500 -75*error_2 
	#k2*error_1 + k3*error_2;

	print('end of line status when no yellow',end_of_line)

	if end_of_line is True:
	    twist_object.linear.x = 0.0
	    twist_object.angular.z = 0.0

        rospy.loginfo("ANGULAR.z VALUE SENT===>"+str(twist_object.angular.z))
	rospy.loginfo("LINEAR.x VALUE SENT===>"+str(twist_object.linear.x))
	
        # Make it start turning
        self.moveTB3_object.move_robot(twist_object)


def shutdown():
	
    rospy.loginfo("Stop TB3")
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    move = Twist()
    move.linear.x = 0.0
    move.angular.z = 0.0
    pub.publish(move)
    #use OM to place item on the ground when end of yellow line
    place()
    rate.sleep()

#########################################################################################
##### Complete function here; this function moves the TB3 to follow the yellow line #####
#########################################################################################

def move():
   ##### Hint: Call the LineFollower class, then call the MoveTB3 class, and lastly configure a suitable condition under a while loop to stop the TB3 when it is at the end of the line.


   



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
    rospy.init_node('Project5_Node', anonymous=True)

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
    ##############################################################################
    ##### To complete function here; this function moves the arm to the left #####
    ##############################################################################
    

    
    return all_close(joint_goal, current_joints, 0.01)

  def Down(self):
    #######################################################################
    ##### To complete function here; this function moves the arm down #####
    #######################################################################




    return all_close(joint_goal, current_joints, 0.01)

  def gripper_open(self):
    ######################################################################
    ##### To complete function here; this function opens the gripper #####  
    ######################################################################



    return all_close(joint_grip, current_grip, 0.01)

  def gripper_close(self):
    #######################################################################
    ##### To complete function here; this function closes the gripper #####
    #######################################################################



    return all_close(joint_grip, current_grip, 0.01)


###################################################################
##### To complete function here; this function picks the item #####
###################################################################

def pick():
  try:
    # Setting up the moveit_commander ...
    move_OM = MoveGroupPythonInteface()

    # Execute movement using 1st joint state goal: to move to left; Complete here to call the function, followed by a delay to allow time for the robot to complete its movements before the next step
    

    # Execute gripper open; Complete here to call the function, followed by a delay to allow time for the robot to complete its movements before the next step
    

    # Execute movement using 2nd joint state goal: to move down; Complete here to call the function, followed by a delay to allow time for the robot to complete its movements before the next step
    

    #Execute gripper close; Complete here to call the function, followed by a delay to allow time for the robot to complete its movements before the next step
    

    # Execute movement using 3rd joint state goal: to move up; Complete here to call the function, followed by a delay to allow time for the robot to complete its movements before the next step
    

    # Execute movement using 4th joint state goal: to face front; Complete here to call the function, followed by a delay to allow time for the robot to complete its movements before the next step
    

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

####################################################################
##### To complete function here; this function places the item #####
####################################################################

def place():
  try:
    # Setting up the moveit_commander ...
    move_OM = MoveGroupPythonInteface()

    # Execute movement using 1st joint state goal; move to left ...
    

    # Execute movement using 2nd joint state goal; move down ...
    

    # Execute gripper open ...
    

    # Execute movement using 3rd joint state goal; move up ...
    

    # Execute movement using 4th joint state goal; face front ...
    

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

  #Move TB3 and item to desired position/orientation
  move()

