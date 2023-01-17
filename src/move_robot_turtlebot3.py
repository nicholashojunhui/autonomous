#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class MoveTB3(object):

    # Assigning Variables
    def __init__(self):
    
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmd_vel_subs = rospy.Subscriber('/cmd_vel', Twist, self.cmdvel_callback)
        self.last_cmdvel_command = Twist()
        self._cmdvel_pub_rate = rospy.Rate(10)


    def cmdvel_callback(self,msg):
        self.last_cmdvel_command = msg

    def move_robot(self, twist_object):
        self.cmd_vel_pub.publish(twist_object)
        self._cmdvel_pub_rate.sleep()
                                    
    def clean_class(self):
        # Stop Robot
	rospy.loginfo("Initiate STOP ROBOT")  
        twist_object = Twist()
        twist_object.angular.z = 0.0
	twist_object.linear.x = 0.0
        self.move_robot(twist_object)
	stop_robot = True
	rospy.loginfo("STOPPING ROBOT")

def main():
    rospy.init_node('move_robot_node', anonymous=True)
    
    stop_robot = False    

    moveTB3_object = MoveTB3()
    twist_object = Twist()
    # Make it start turning
    twist_object.angular.z = 0.5

    rate = rospy.Rate(10)
    
    print('stop robot status?',stop_robot)

    while not stop_robot:
        moveTB3_object.move_robot(twist_object)
        rate.sleep()

    
if __name__ == '__main__':
    main()
