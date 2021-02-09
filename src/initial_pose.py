#!/usr/bin/env python
 
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
 
rospy.init_node('init_pos')
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 10)

rospy.sleep(3)
checkpoint = PoseWithCovarianceStamped()

# Use $ rostopic echo /odom -n1 to obtain poses (both position and orientation)

checkpoint.pose.pose.position.x = -1.99968900545
checkpoint.pose.pose.position.y = -0.499122759107
checkpoint.pose.pose.position.z = -0.0010073990697
 
[x,y,z,w]=quaternion_from_euler(0.0,0.0,0.0)
checkpoint.pose.pose.orientation.x = -6.03836998093e-06
checkpoint.pose.pose.orientation.y = 0.00158963960308
checkpoint.pose.pose.orientation.z = 0.00275331003065
checkpoint.pose.pose.orientation.w = 0.999994946134
 
print checkpoint
pub.publish(checkpoint)
	
