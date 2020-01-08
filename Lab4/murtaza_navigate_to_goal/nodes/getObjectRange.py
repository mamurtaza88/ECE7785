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
        #self.objectrange_sub1 = rospy.Subscriber('imageLocation',Point,self.ball_callback)
        self.objectrange_sub2 = rospy.Subscriber('scan',LaserScan,self.LIDAR_callback)
        if VERBOSE :
            print "subscribed to  Laser"


    def LIDAR_callback(self, ros_data):

        lidar_value = ros_data.ranges
        front = lidar_value[0]
        side  = lidar_value[90]

        if front > 0.4: 
            lidar_pub = Point()
            lidar_pub.x = 999
            lidar_pub.y = 999
            lidar_pub.z = 999
            self.objectrange_pub.publish(lidar_pub)

        else:
            lidar_pub = Point()
            lidar_pub.x = front
            lidar_pub.y = 999
            lidar_pub.z = 999
            self.objectrange_pub.publish(lidar_pub)
                        
                    
    
            



def main(args):
    '''Initializes and cleanup ros node'''
    print "Program Started Get Object Range"
    
    rospy.init_node('getObjectRange', anonymous=True)
    ic = getObjectRange()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down OS getObjectRange"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
