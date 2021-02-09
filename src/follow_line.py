#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot_turtlebot3 import MoveTB3


cx = 0
cy = 0

class LineFollower(object):

    # Assigning Variables
    def __init__(self):
    
        self.bridge_object = CvBridge()
        #self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",Image,self.camera_callback)
	self.image_sub = rospy.Subscriber("/raspicam_node/image",Image,self.camera_callback)
        self.moveTB3_object = MoveTB3()

    # function to detect if there is a yellow line within camera visual range 
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
        
        # Draw the centroid in the resultant image
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
	
	# update errors
	error_2 = error_1
	error_1 = error_x 	
        error_x = cx - width / 2;	
	a = error_x

        twist_object = Twist();

	# Move TB3 forward
        twist_object.linear.x = 0.1;

        #kp = -1/500
 	#kd = 1
	#ki = 0
	#k1 =kp + ki + kd
	#k2 = -kp -2*kd
	#k3 = kd
	print('0',error_x,'1',error_1,'2',error_2)

####### Simple math algorithm to calculate required angular velocity to enable TB3 to follow the yellow line at bends #######  
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
    rate.sleep()


def main():
    rospy.init_node('line_following_node', anonymous=True)
    error_1 = 0
    error_2 = 0
    error_x = 0

    # Call the LineFollower class constructed earlier
    line_follower_object = LineFollower()

    # Call the MoveTB3 class constructed in move_robot_turtlebot3.py
    moveTB3_object = MoveTB3()
    
    global rate
    rate = rospy.Rate(10)

    global end_of_line
    end_of_line = False
  
    while not end_of_line:

	# Condition to stop the TB3 at the end of the yellow line  
	if cx < 250 and cy > 300:
	    rospy.loginfo("Initiate STOP PROGRAMME")        
	    moveTB3_object.clean_class()
            cv2.destroyAllWindows()
	    rospy.loginfo("shutdown time!")
	    end_of_line = True
	    print('end of line status when no yellow',end_of_line)
	    rospy.on_shutdown(shutdown)

	else:
            rate.sleep()

    
if __name__ == '__main__':
    main()
