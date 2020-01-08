#!/usr/bin/env python  
# Python libs
import sys, time

# numpy and scipy
import numpy as np
#from scipy.ndimage import filters

# Ros libraries
import roslib
import rospy

# Ros Messages
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist


VERBOSE=True

class drive_wheels:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.wheels_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        # self.bridge = CvBridge()

        # subscribed Topic
        self.wheels_sub = rospy.Subscriber('/Ball_Position',Point,self.callback)
        if VERBOSE :
            print "subscribed to  Ball_Position"

    def callback(self, ros_data):
    	
    	
    	ref_position = ros_data.x
    	vel_msg = Twist();


    	vel_msg.linear.x = 0
    	vel_msg.linear.y = 0
    	vel_msg.linear.z = 0
    	vel_msg.angular.x = 0
    	vel_msg.angular.y = 0

    	if ref_position  == 0:
    		print "Do Nothing"
    		vel_msg.angular.z = 0	 

    	elif ref_position > 480/2:
    	 	print "move Left"
    	 	vel_msg.angular.z = -0.2
    	
    	elif ref_position < 480/2:
    	 	print "move Right"
    		vel_msg.angular.z = 0.2	

    	# Publish the velocities
    	self.wheels_pub.publish(vel_msg)
    	


def main(args):
    '''Initializes and cleanup ros node'''
    print "Program Started drive_wheels"
    ic = drive_wheels()
    rospy.init_node('drive_wheels', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down OS drive_wheels node"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

