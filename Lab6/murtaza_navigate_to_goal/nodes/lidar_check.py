#!/usr/bin/env python  
# Python libsfrom nav_msgs.msg import Odometry 
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
from nav_msgs.msg import Odometry 


VERBOSE=True

class goToGoal:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.goToGoal_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # self.bridge = CvBridge()

        # subscribed Topic

        self.obstacle_detected=9999
        self.goToGoal_sub1 = rospy.Subscriber('/Object_Range',Point,self.detectObstacle_callback)
        self.goToGoal_sub2 = rospy.Subscriber('/odom',Odometry,self.follow_callback)
        
        self.waypointStatus = 1
        self.waypointStatus_inner = 1
        self.waypointStatus_inner1 = 1
        self.waypointStatus_inner_4 = 1
        self.stop_flag=False
        self.stop_flag2 = False
        self.stop_flag3 = False
        self.goToGoal_flag = 0
        self.counter=0
        self.globalPos = Point()
        self.globalAng = 0.0
        self.Init = True
        self.Init_ang = 0.0
        self.Init_pos = Point()
        self.object_done = False
        self.now_move=False

        if VERBOSE :
            print "subscribed to  goToGoal"


    def detectObstacle_callback(self, ros_data):
        # print "Entered detectObstacle_callback"

        ref_position = ros_data.x
        self.obstacle_detected = ref_position
       
    def follow_callback(self, ros_data):
        # print "Entered follow_callback"
        Odom = ros_data 
        self.update_Odometry(Odom)  
        
        # print "Exited update_Odometry"

        current_position_x = self.globalPos.x
        current_position_y = self.globalPos.y

        current_Orientation = self.globalAng
        vel_msg = Twist()
        kp_x = 0.5
        kp_angle = 0.65

        # print('lidar is: ', self.obstacle_detected)
        if self.obstacle_detected<900 and self.obstacle_detected>0 and not self.object_done:
            self.object_done = True
            print("Object. turn 90")
            
           
            # print("Current_Orientation =",current_Orientation)
        
        if self.object_done:
            ref_angle = math.pi/2 
            error_angle = current_Orientation-ref_angle
            if abs(error_angle) > 0.1: 
                    # print("Reference_Orientation =",ref_angle)
                    # print("Current_Orientation =",current_Orientation)
                    # print("Reference_Position_x =",ref_position_x)
                    # print("Current_Position_x =",current_position_x)
                    # print("Reference_Position_y =",ref_position_y)
                    # print("Current_Position_y =",current_position_y)
                    pid_angle = -kp_angle*error_angle
                
                    vel_msg.linear.x = 0                
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0        
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = pid_angle
                    self.goToGoal_pub.publish(vel_msg)
            else:
                # self.object_done=False
                vel_msg.linear.x = 0                
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0        
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0
                self.goToGoal_pub.publish(vel_msg)
                self.now_move=True
                

        if self.now_move:
            error_dist = current_position_y - 0.6
            if abs(error_dist) > 0.1: 
                pid = -kp_x*error_dist
            
                vel_msg.linear.x = pid                
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0        
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0
                self.goToGoal_pub.publish(vel_msg)
            else:
                vel_msg.linear.x = 0               
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0        
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0
                self.goToGoal_pub.publish(vel_msg)
                self.now_move=False




        

    

    def update_Odometry(self, Odom):
        # print "Entered update_Odometry"
        
        position = Odom.pose.pose.position
        
        #Orientation uses the quaternion aprametrization.
        #To get the angular position along the z-axis, the following equation is required.
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init:
            #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            # print "Entered self.Init due to True"
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z

        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        

        #We subtract the initial values
        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang
        
    
    

def main(args):
    '''Initializes and cleanup ros node'''
    print "Program Started drive_wheels"
    rospy.init_node('goToGoal', anonymous=True)
    ic = goToGoal()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down OS chase_object node"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

