#!/usr/bin/env python  
# Python libs
import sys, time, math

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

class chase_object:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.chase_object_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # self.bridge = CvBridge()

        # subscribed Topic
        self.chase_object_sub = rospy.Subscriber('/Object_Range',Point,self.callback)
        if VERBOSE :
            print "subscribed to  chase_object"

    def callback(self, ros_data):
    	
    	
    	ref_position = ros_data.x
    	ref_angle = ros_data.y
        vel_msg = Twist()
        
        vel_msg.linear.x = 0
            
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0        
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        ref_angle_rad = ref_angle*math.pi/180;
        print(ref_angle)
        if ref_angle < 900:
            if ref_angle_rad < 0.11 and ref_angle_rad > -0.11:
                kp_x = 0
                error = 0.5 - ref_position
                pid_x = 0

                if abs(error) > 0.1: 
                    kp_x = -0.32
                    pid_x = kp_x*error

                vel_msg.linear.x = pid_x
                
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0        
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0

                print "velocities linear = " , pid_x
                print("Ang:", ref_angle_rad)

        	# if ref_position > 0:
        	#  	print "move Left"
        	 	
        	
        	# elif ref_position < 0:
        	#  	print "move Right"
        	# 	vel_msg.angular.z = pid_angle

        	# Publish the velocities
            else:
                print("Ang:", ref_angle_rad)
                kp_angle = 0.65
                pid_angle = kp_angle*ref_angle_rad

                
                vel_msg.linear.x = 0
                
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0        
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = pid_angle

                print "velocities linear angular = " , 0
                print "velocities angular  angular = " , pid_angle


            self.chase_object_pub.publish(vel_msg)
            print "Twist vel_msg = ", vel_msg
    	else:
            vel_msg.linear.x = 0
            
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0        
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            print "velocities linear not detected = " , 0
            print "velocities angular not detected = " , 0
            self.chase_object_pub.publish(vel_msg)

def main(args):
    '''Initializes and cleanup ros node'''
    print "Program Started drive_wheels"
    ic = chase_object()
    rospy.init_node('chase_object', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down OS chase_object node"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

