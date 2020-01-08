#!/usr/bin/env python  
# Python libs
import sys, time

# numpy and scipy
import numpy as np
#from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point


VERBOSE=True

class find_ball:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.img_pub = rospy.Publisher('Ball_Position', Point, queue_size=5)
        # self.bridge = CvBridge()

        # subscribed Topic
        self.img_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,self.callback)
        if VERBOSE :
            print "subscribed to /camera/image/compressed"

    def callback(self, ros_data):
        # '''Callback function of subscribed topic. 
        # Here images get converted and features detected'''
        # if VERBOSE :
        #     print 'received image of type: "%s"' % ros_data.format

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        #image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        
        cv2.waitKey(1)
        gray = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
        normalizedImg = np.zeros((800, 800))
        cv2.normalize(gray,  normalizedImg, 0, 255, cv2.NORM_MINMAX)
        rows = gray.shape[0]
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1,  rows / 8, param1=90, param2=40, minRadius=10, maxRadius=110)
        #frame = cv2.flip(gray,0)
        
        cent_x = 0
        cent_y = 0
        radius = 0

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0], i[1])
                # circle center
                cv2.circle(image_np, center, 1, (0, 100, 100), 3)
                # circle outline
                radius = i[2]
                cv2.circle(image_np, center, radius, (255, 0, 255), 3)
                cent_x = i[0]
                cent_y = i[1]

                center = (cent_x, cent_y)
                print 'Pixel Location', center
        

       #cv.imshow("detected circles", src)
        # cv2.imshow('Color Frame',image_np)
        # write the flipped frame
        font = cv2.FONT_HERSHEY_SIMPLEX
        #font = cv2.FONT_HERSHEY_SIMPLEX
        #display_text = str(i[0])+','+str(i[1])
        display_text = str(cent_x)+','+str(cent_y)

        #cv2.putText(gray,display_text, (i[0], i[1]), font, 1,(255,255,255),1)
        cv2.putText(image_np,display_text, (cent_x, cent_y), font, 1,(255,255,255),1)
        
        
        #out.write(gray)
        
       # cv2.imshow('normalized Image',normalizedImg)
        #cv2.imshow('detected circles',gray)

        # create Point class to publish
        center = Point()
        center.x = cent_x
        center.y = cent_y

        self.img_pub.publish(center)


def main(args):
    '''Initializes and cleanup ros node'''
    print 'Program Started'
    ic = find_ball()
    rospy.init_node('find_ball', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS find_ball node"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)