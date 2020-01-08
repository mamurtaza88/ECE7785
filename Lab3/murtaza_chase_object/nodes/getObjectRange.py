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
from sensor_msgs.msg import LaserScan


VERBOSE=True

class getObjectRange:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.objectrange_pub = rospy.Publisher('Object_Range', Point, queue_size=5)
        # self.bridge = CvBridge()

        # subscribed Topic
        self.objectrange_sub1 = rospy.Subscriber('imageLocation',Point,self.ball_callback)
        self.objectrange_sub2 = rospy.Subscriber('scan',LaserScan,self.LIDAR_callback)
        if VERBOSE :
            print "subscribed to  Ball_Position"
            print "subscribed to  Laser"



    def ball_callback(self, ros_data):
    	
    	
    	ref_position = (ros_data.x)
        ref_width = ros_data.y
        ref_frame = ros_data.z
        if ref_position < 900:
            ref_position = ref_frame/2-ros_data.x

        
        self.ball_pos = ref_position
        self.ball_rad = ref_width
        self.frame = ref_frame
        vel_msg = Twist();
   	

    def LIDAR_callback(self, ros_data):

        if self.ball_pos < 900:
            lidar_value = ros_data.ranges
            theta_Actual = self.ball_pos*(62.2/self.frame)
            width_object = self.ball_rad-30
            print("width_object = ", width_object)
            w_degree = width_object*(62.2/self.frame)
            d_lidar_left_camFrame = int(math.floor(theta_Actual+w_degree/2))
            d_lidar_right_camFrame = int(math.floor(theta_Actual-w_degree/2))
            d_actual = 0; k = 0;
            print("lidar_value[0] = " ,lidar_value[0])
            print("d_lidar_left_camFrame = ",d_lidar_left_camFrame)
            print("d_lidar_right_camFrame = ",d_lidar_right_camFrame)  
            for x in range(d_lidar_right_camFrame, d_lidar_left_camFrame):
                if x < 0:
                    x = x + 360
                  
                d_actual = d_actual + lidar_value[x]
                k += 1
                
                
            if k!= 0:   
                d_actual = d_actual/k

            lidar_pub = Point()
            lidar_pub.x = d_actual
            lidar_pub.y = theta_Actual
            lidar_pub.z = 0

            lidar_value_print = (d_actual, theta_Actual)
            print "Value of Lidar = ", lidar_value_print
            self.objectrange_pub.publish(lidar_pub)
        else:
            print("Object Not Detected")
            lidar_pub = Point()
            lidar_pub.x = 999
            lidar_pub.y = 999
            lidar_pub.z = 999
            self.objectrange_pub.publish(lidar_pub)



def main(args):
    '''Initializes and cleanup ros node'''
    print "Program Started Get Object Range"
    ic = getObjectRange()
    rospy.init_node('getObjectRange', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down OS getObjectRange"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
