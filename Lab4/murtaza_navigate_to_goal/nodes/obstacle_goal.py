#!/usr/bin/env python
# Python libsfrom nav_msgs.msg import Odometry
import sys, time, math

# numpy and scipy
import numpy as np
# from scipy.ndimage import filters

# Ros libraries
import roslib
import rospy

# Ros Messages
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

VERBOSE = True


class goToGoal:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.goToGoal_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # self.bridge = CvBridge()

        # subscribed Topic
        self.now_goto_goal = False
        self.obstacle_detected = 0
        self.goToGoal_sub1 = rospy.Subscriber('/Object_Range', Point, self.detectObstacle_callback)
        self.goToGoal_sub2 = rospy.Subscriber('/odom', Odometry, self.follow_callback)

        self.go_straight = True
        self.first_step = True
        self.waypointStatus = 1
        self.waypointStatus_inner = 1
        self.waypointStatus_inner1 = 1
        self.waypointStatus_inner_4 = 1
        self.stop_flag = False
        self.stop_flag2 = False
        self.stop_flag6 = False
        self.stop_flag3 = False
        self.counter = 0
        self.globalPos = Point()
        self.globalAng = 0.0
        self.Init = True
        self.Init_ang = 0.0
        self.all_done = False
        self.object_done = False
        self.now_move = False
        self.detected_obstacle_pos_y = 0
        self.Init_pos = Point()
        self.now_goal = False
        # self.obstacle_detected=0

        if VERBOSE:
            print
            "subscribed to  goToGoal"

    def detectObstacle_callback(self, ros_data):
        print
        "Entered detectObstacle_callback"

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
        if current_Orientation < 0:
            current_Orientation += 2 * math.pi

        vel_msg = Twist()
        kp_x = 0.5
        kp_angle = 0.65

        if self.waypointStatus == 1:
            self.counter += 1
            print("Entering WaypointStatus = ", self.waypointStatus)
            ref_position_x = 1.78
            ref_position_y = 0
            # ref_angle = 0;
            # print("Reference_Orientation =",ref_angle)
            # print("Current_Orientation =",current_Orientation)
            # print("Reference_Position_x =",ref_position_x)
            # print("delta_x=", ref_position_x - current_position_x)

            error = math.sqrt((current_position_x - ref_position_x) ** 2)
            if current_position_x > ref_position_x:
                error = -error
            # error = current_position_x-ref_position_x

            ref_angle = np.arctan2(ref_position_y - current_position_y, ref_position_x - current_position_x)

            if ref_angle < 0:
                ref_angle += 2*np.pi
            print('Ref ang is: ', ref_angle)
            print('Cur orient: ', current_Orientation)
            error_angle = current_Orientation - ref_angle
            # print("delta_ang =", error_angle)
            print("ang_err= ", error_angle)

            if error_angle > np.pi/2:
                error_angle-=2*np.pi

            # if abs(error) < 0.12:
            #     error_angle = 0

            # if abs(error_angle) < 0.015:
            #     error_angle = 0

            if abs(error) < 0.05:
                self.stop_flag = True


            if error_angle > 0.05:
                # Only set angle first
                print("setting angle")
                pid_x = 0
                pid_angle = kp_angle*error_angle
                vel_msg.linear.x = pid_x

                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = pid_angle

                self.goToGoal_pub.publish(vel_msg)

            elif error > 0.05:
                print("setting distance")
                pid_x = kp_x*error
                pid_angle = 5 * kp_angle * error_angle
                vel_msg.linear.x = pid_x

                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = pid_angle

                self.goToGoal_pub.publish(vel_msg)

            else:
                # Reached at point B
                print('Reached point B')

                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0
                self.goToGoal_pub.publish(vel_msg)
                time.sleep(2)
                # for x in range(1, 1, 2000):
                #     vel_msg.linear.x = 0
                #     vel_msg.linear.y = 0
                #     vel_msg.linear.z = 0
                #     vel_msg.angular.x = 0
                #     vel_msg.angular.y = 0
                #     vel_msg.angular.z = 0
                #     # print "velocities linear not detected = " , 0
                #     # print "velocities angular not detected = " , 0
                #     self.goToGoal_pub.publish(vel_msg)
                self.waypointStatus = 2

        elif self.waypointStatus == 2:
            print("Entering WaypointStatus = ", self.waypointStatus)

            if self.waypointStatus_inner == 1:
                # Turn segment
                print("Turning to avoid obstace Part 1")
                ref_position_x = 2.2
                ref_position_y = 0.6
                ref_angle = np.arctan2(ref_position_y - current_position_y,
                                       ref_position_x - current_position_x)  # math.pi/2 -
                error_angle = current_Orientation - ref_angle
                print("ange_delta =", error_angle)
                # print("Current_Orientation =",current_Orientation)

                if abs(error_angle) > 0.05:
                    # print("Reference_Orientation =",ref_angle)
                    # print("Current_Orientation =",current_Orientation)
                    # print("Reference_Position_x =",ref_position_x)
                    # print("Current_Position_x =",current_position_x)
                    # print("Reference_Position_y =",ref_position_y)
                    # print("Current_Position_y =",current_position_y)
                    pid_angle = -kp_angle * error_angle

                    vel_msg.linear.x = 0
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = pid_angle
                    self.goToGoal_pub.publish(vel_msg)
                else:
                    self.waypointStatus_inner = 0

            if self.waypointStatus_inner == 0:
                print("Moving to avoid obstace Part 1")  ########################

                ref_position_x = 2.4
                ref_position_y = 0.6
                dist1 = current_position_x - ref_position_x
                dist2 = current_position_y - ref_position_y
                print("delta_x way2=", ref_position_x - current_position_x)
                # error = dist1+dist2
                error = math.sqrt((current_position_x - ref_position_x) ** 2)
                ref_angle = np.arctan2(ref_position_y - current_position_y,
                                       ref_position_x - current_position_x)  # math.pi/2 -
                ref_angle = math.pi / 4
                error_angle = current_Orientation - ref_angle

                print("error way2=", error)
                print("ang_delta=", error_angle)

                if abs(error) < 0.1:
                    error_angle = 0

                # print("error_angle = ", error_angle)
                if current_position_x > ref_position_x:
                    error = -error

                if abs(error) < 0.1:
                    self.stop_flag2 = True
                # if abs(error) > 0.01:
                if not self.stop_flag2:
                    pid_x = kp_x * error
                    pid_angle = 0
                    vel_msg.linear.x = pid_x

                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = pid_angle
                    # print("Reference_Orientation =",ref_angle)
                    # print("Current_Orientation =",current_Orientation)
                    # print("Reference_Position_x =",ref_position_x)
                    # print("Current_Position_x =",current_position_x)
                    # print("Reference_Position_y =",ref_position_y)
                    # print("Current_Position_y =",current_position_y)
                    self.goToGoal_pub.publish(vel_msg)
                else:
                    self.waypointStatus_inner = 3

            elif self.waypointStatus_inner == 3:
                print('self wypoint 3')
                ref_position_x = 1.5
                ref_position_y = 1.5
                ref_angle = np.arctan2(ref_position_y - current_position_y,
                                       ref_position_x - current_position_x) +0.1  # math.pi/2 - np.arctan2(0.7,0.5)
                # ref_angle = 2*math.pi/3
                error_angle = current_Orientation - ref_angle
                print('ang_delta: ', error_angle)
                # print("Reference_Orientation =",ref_angle)
                # print("Current_Orientation =",current_Orientation)

                # if abs(error_angle) > 0.1 and self.waypointStatus_inner1 == 1:
                if abs(error_angle) > 0.1:
                    print("Turning to avoid obstace Part 2")
                    # print("Reference_Orientation =",ref_angle)
                    # print("Current_Orientation =",current_Orientation)
                    # print("Reference_Position_x =",ref_position_x)
                    # print("Current_Position_x =",current_position_x)
                    # print("Reference_Position_y =",ref_position_y)
                    # print("Current_Position_y =",current_position_y)
                    pid_angle = -kp_angle * error_angle

                    vel_msg.linear.x = 0
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = pid_angle
                    self.goToGoal_pub.publish(vel_msg)
                else:
                    self.waypointStatus_inner = 4

            elif self.waypointStatus_inner == 4:
                # check this
                print("Moving towards Goal2")
                self.waypointStatus_inner1 == 2
                ref_position_x = 1.5
                ref_position_y = 1.5
                dist1 = current_position_x - ref_position_x
                dist2 = current_position_y - ref_position_y
                # error = dist1+dist2
                error = math.sqrt((current_position_x - ref_position_x) ** 2)
                ref_angle = np.arctan2(ref_position_y - current_position_y,
                                       ref_position_x - current_position_x)  # math.pi/2 -
                error_angle = current_Orientation - ref_angle
                print("delta_x way4=", ref_position_x - current_position_x)
                print("delta_ang way4=", error_angle)
                # if abs(error)<0.1:
                #     self.stop_flag2=True

                # if abs(error)<0.12:
                #     error_angle=0

                # if abs(error_angle)<0.015:
                #     error_angle = 0

                # self.counter+=1

                # if abs(error) > 0.01:
                if error > 0.1:
                    # self.stop_flag2
                    # if self.counter%2==0:
                    #     error_angle=0
                    # else:
                    #     error=0
                    pid_x = kp_x * error
                    pid_angle = -kp_angle * error_angle
                    vel_msg.linear.x = pid_x

                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = pid_angle
                    # print("Reference_Orientation =",ref_angle)
                    # print("Current_Orientation =",current_Orientation)
                    # print("Reference_Position_x =",ref_position_x)
                    # print("Current_Position_x =",current_position_x)
                    # print("Reference_Position_y =",ref_position_y)
                    # print("Current_Position_y =",current_position_y)
                    self.goToGoal_pub.publish(vel_msg)
                else:
                    print('stopping')

                    vel_msg.linear.x = 0
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = 0
                    # print "velocities linear not detected = " , 0
                    # print "velocities angular not detected = " , 0
                    self.goToGoal_pub.publish(vel_msg)
                    # time.sleep(5)
                    self.waypointStatus = 3
                    print('Pos x: ', current_position_x)
                    print('Pos y: ', current_position_y)
                    # time.sleep(15)


        elif self.waypointStatus == 3:
            # Reached at point D
            # Orientation correct
            ref_position_x = 0
            ref_position_y = 1.3
            ref_angle = np.arctan2(ref_position_y - current_position_y,
                                   ref_position_x - current_position_x)  # math.pi/2 - np.arctan2(0.7,0.5)
            # ref_angle = np.pi-0.1
            if ref_angle < 0:
                ref_angle += 2 * np.pi
            error_angle = current_Orientation - ref_angle
            print("Reference_Orientation =", ref_angle)
            # if current_Orientation<0:
            #     current_Orientation += 2*np.pi

            print("Current_Orientation =", current_Orientation)

            # ref_angle = np.pi
            # if abs(error_angle) > 0.1 and self.waypointStatus_inner1 == 1:
            if error_angle > 0.1 or error_angle < -0.1:
                print("Turning towards goal Part 3")
                # print("Reference_Orientation =",ref_angle)
                # print("Current_Orientation =",current_Orientation)
                # print("Reference_Position_x =",ref_position_x)
                # print("Current_Position_x =",current_position_x)
                # print("Reference_Position_y =",ref_position_y)
                # print("Current_Position_y =",current_position_y)
                pid_angle = -kp_angle * error_angle

                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = pid_angle
                self.goToGoal_pub.publish(vel_msg)

            else:
                self.waypointStatus = 4


        elif self.waypointStatus == 4:

            # vel_msg.linear.x = 0
            # vel_msg.linear.y = 0
            # vel_msg.linear.z = 0
            # vel_msg.angular.x = 0
            # vel_msg.angular.y = 0
            # vel_msg.angular.z = 0
            # self.goToGoal_pub.publish(vel_msg)

            # time.sleep(2)
            # heyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy
            if self.waypointStatus_inner_4 == 1:
                # Turn segment
                # print("Turning to alin with goal")
                # ref_position_x = 0
                # ref_position_y = 1.87
                # ref_angle = np.arctan2(ref_position_y-current_position_y,ref_position_x-current_position_x)#math.pi/2 -
                # error_angle = current_Orientation-ref_angle
                # print("ange_delta =",error_angle)
                # # print("Current_Orientation =",current_Orientation)

                # if abs(error_angle) > 0.1:
                #     # print("Reference_Orientation =",ref_angle)
                #     # print("Current_Orientation =",current_Orientation)
                #     # print("Reference_Position_x =",ref_position_x)
                #     # print("Current_Position_x =",current_position_x)
                #     # print("Reference_Position_y =",ref_position_y)
                #     # print("Current_Position_y =",current_position_y)
                #     pid_angle = -0.2*kp_angle*error_angle

                #     vel_msg.linear.x = 0
                #     vel_msg.linear.y = 0
                #     vel_msg.linear.z = 0
                #     vel_msg.angular.x = 0
                #     vel_msg.angular.y = 0
                #     vel_msg.angular.z = pid_angle
                #     self.goToGoal_pub.publish(vel_msg)
                # else:
                self.waypointStatus_inner_4 = 0


            elif self.waypointStatus_inner_4 == 0:

                # print('Yayyyyyy!!!!')

                # print("obstacle_detected: ", self.obstacle_detected)

                if self.obstacle_detected < 900 and self.obstacle_detected > 0 and not self.object_done:
                    print("obstacle_detected: ", self.obstacle_detected)
                    self.detected_obstacle_pos_y = current_position_y
                    self.object_done = True
                    self.go_straight = False

                if not self.all_done:
                    print("Going into obstacle mode")
                    self.if_obstacle(current_Orientation, current_position_y, current_position_x, kp_angle, kp_x)

                # vel_msg.linear.x = 0
                # vel_msg.linear.y = 0
                # vel_msg.linear.z = 0
                # vel_msg.angular.x = 0
                # vel_msg.angular.y = 0
                # vel_msg.angular.z = 0
                # self.goToGoal_pub.publish(vel_msg)
                # time.sleep(15)
                if self.go_straight:
                    self.counter += 1
                    print("Entering WaypointStatus = ", self.waypointStatus)
                    ref_position_x = 0
                    ref_position_y = 1.7
                    ref_angle = 0;
                    # print("Reference_Orientation =",ref_angle)
                    # print("Current_Orientation =",current_Orientation)
                    # print("Reference_Position_x =",ref_position_x)
                    print("delta_x=", ref_position_x - current_position_x)

                    # print("Current_Position_y =",current_position_y
                    print("curr_y = ", current_position_y)
                    print("curr_x = ", current_position_x)

                    error = math.sqrt((current_position_x - ref_position_x) ** 2)
                    if current_position_x > ref_position_x:
                        error = -error
                    # error = current_position_x-ref_position_x
                    error_angle = np.arctan2(ref_position_y - current_position_y,
                                             ref_position_x - current_position_x) - np.pi

                    # if (current_position_y-ref_position_y)>0:
                    #     error_angle = math.pi - error_angle

                    # print("delta_x =",error)
                    print("ang_err= ", error_angle)
                    # error_angle=0
                    if abs(error) < 0.12:
                        error_angle = 0

                    if abs(error_angle) < 0.015:
                        error_angle = 0

                    if abs(error) < 0.05:
                        self.stop_flag3 = True

                    if not self.stop_flag3:
                        if self.counter % 2 == 0:
                            error_angle = 0
                        else:
                            error = 0
                        print('Publishing velocity')
                        pid_x = -kp_x * error
                        pid_angle = 5 * kp_angle * error_angle
                        pid_angle = 0
                        vel_msg.linear.x = pid_x

                        vel_msg.linear.y = 0
                        vel_msg.linear.z = 0
                        vel_msg.angular.x = 0
                        vel_msg.angular.y = 0
                        vel_msg.angular.z = pid_angle

                        self.goToGoal_pub.publish(vel_msg)

                    else:
                        # Reached at point B
                        print('Reached point D')
                        time.sleep(15)

    def if_obstacle(self, current_orientation, current_position_y, current_position_x, kp_angle, kp_x):

        # print("Current_Orientation =",current_Orientation)
        vel_msg = Twist()
        if self.object_done and self.first_step:
            ref_angle = math.pi / 2
            error_angle = current_orientation - ref_angle
            print("obs error angle: ", error_angle)
            if abs(error_angle) > 0.05:
                # print("Reference_Orientation =",ref_angle)
                # print("Current_Orientation =",current_Orientation)
                # print("Reference_Position_x =",ref_position_x)
                # print("Current_Position_x =",current_position_x)
                # print("Reference_Position_y =",ref_position_y)
                # print("Current_Position_y =",current_position_y)
                pid_angle = -kp_angle * error_angle

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
                self.now_move = True
                self.first_step = False

        if self.now_move:
            # go forward
            error_dist = current_position_y - (self.detected_obstacle_pos_y + 0.7)
            print("rec err dist: ", error_dist)
            if abs(error_dist) > 0.1:
                pid = -kp_x * error_dist

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
                self.now_move = False
                self.now_goal = True
                self.all_done = False
                print('Success in obstacle avoid')
                # time.sleep(2)
        elif self.now_goal:
            print('Now go back to goal')
            # Current your angle:
            # time.sleep(15)
            if not self.now_goto_goal:
                # FIrst align with goal
                print("Aligning with final Goal")
                ref_position_x = 0
                ref_position_y = 1.1
                ref_angle = np.arctan2(ref_position_y - current_position_y,
                                       ref_position_x - current_position_x)  # math.pi/2 -
                if ref_angle < 0:
                    ref_angle += 2 * np.pi
                print("Now angle is: ", ref_angle)

                error_angle = current_orientation - ref_angle
                print("ange_delta =", error_angle)
                # print("Current_Orientation =",current_Orientation)

                if abs(error_angle) > 0.05:
                    # print("Reference_Orientation =",ref_angle)
                    # print("Current_Orientation =",current_Orientation)
                    # print("Reference_Position_x =",ref_position_x)
                    # print("Current_Position_x =",current_position_x)
                    # print("Reference_Position_y =",ref_position_y)
                    # print("Current_Position_y =",current_position_y)
                    pid_angle = -kp_angle * error_angle

                    vel_msg.linear.x = 0
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = pid_angle
                    self.goToGoal_pub.publish(vel_msg)
                else:
                    print("Align done")
                    self.now_goto_goal = True
                    # self.waypointStatus_inner_obs = 1
            else:
                # Now go straight to goal
                print("Now straight to goal")
                ref_position_x = 0
                ref_position_y = 0.9
                # error = dist1+dist2
                error = math.sqrt((current_position_x - ref_position_x) ** 2)
                ref_angle = np.arctan2(ref_position_y - current_position_y,
                                       ref_position_x - current_position_x)  # math.pi/2 -
                # ref_angle = math.pi / 4
                error_angle = current_orientation - ref_angle

                print("dist_delta =", error)
                print("ang_delta=", error_angle)

                if abs(error) < 0.1:
                    error_angle = 0

                # print("error_angle = ", error_angle)
                if current_position_x > ref_position_x:
                    error = -error

                if abs(error) < 0.1:
                    self.stop_flag6 = True
                # if abs(error) > 0.01:
                if not self.stop_flag6:
                    pid_x = -kp_x * error
                    pid_angle = 0
                    vel_msg.linear.x = pid_x

                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = pid_angle
                    # print("Reference_Orientation =",ref_angle)
                    # print("Current_Orientation =",current_Orientation)
                    # print("Reference_Position_x =",ref_position_x)
                    # print("Current_Position_x =",current_position_x)
                    # print("Reference_Position_y =",ref_position_y)
                    # print("Current_Position_y =",current_position_y)
                    self.goToGoal_pub.publish(vel_msg)
                else:
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = 0
                    self.goToGoal_pub.publish(vel_msg)
                    print("Reached point D after obstacle -YAYYYYYYY")
                    time.sleep(20)

            self.object_done = False

        # return self.all_done

    def move_forward(self, x, y, theta, current_position_x, current_position_y, current_orientation, angle_off=False):
        self.first = False
        vel_msg = Twist()
        self.counter += 1
        # print("Entering WaypointStatus = ", self.waypointStatus)
        ref_position_x = x
        ref_position_y = y
        ref_angle = theta
        # print("Reference_Orientation =",ref_angle)
        print("Current_Orientation =", current_orientation)
        # print("Reference_Position_x =", ref_position_x)
        # print("delta_x=", ref_position_x - self.current_position_x)

        # print("Current_Position_y =",current_position_y)

        error = math.sqrt(
            (current_position_x - ref_position_x) ** 2 + (current_position_y - ref_position_y) ** 2)
        # if self.current_position_x > ref_position_x:
        #     error = -error
        # error = current_position_x-ref_position_x
        curr_angle = np.arctan2(ref_position_y - current_position_y, ref_position_x - current_position_x)
        if curr_angle < -np.pi / 2:
            curr_angle += 2 * np.pi

        error_angle = curr_angle - ref_angle
        if error_angle > np.pi:
            curr_angle -= 2 * np.pi
        print("delta_ang =", error_angle)
        print("delta_dist= ", error)

        # if abs(error) < 0.12:
        #     error_angle = 0
        if angle_off:
            error_angle = 0
        if abs(error_angle) < 0.015:
            error_angle = 0

        if abs(error) < 0.1:
            self.stop_flag = True

        if not self.stop_flag:
            if self.counter % 2 == 0 and not angle_off:
                error = 0
            else:
                error_angle = 0
            pid_x = 5 * self.kp_x * error
            pid_angle = 5 * self.kp_angle * error_angle
            vel_msg.linear.x = pid_x

            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = pid_angle

            self.goToGoal_pub.publish(vel_msg)

        else:
            # Reached at point B
            print('Reached point B')
            self.stop_flag = False
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            self.goToGoal_pub.publish(vel_msg)
            time.sleep(2)
            # for x in range(1, 1, 2000):
            #     vel_msg.linear.x = 0
            #     vel_msg.linear.y = 0
            #     vel_msg.linear.z = 0
            #     vel_msg.angular.x = 0
            #     vel_msg.angular.y = 0
            #     vel_msg.angular.z = 0
            #     # print "velocities linear not detected = " , 0
            #     # print "velocities angular not detected = " , 0
            #     self.goToGoal_pub.publish(vel_msg)
            self.waypointStatus += 1
        return self.current_orientation

    def update_Odometry(self, Odom):
        # print "Entered update_Odometry"

        position = Odom.pose.pose.position

        # Orientation uses the quaternion aprametrization.
        # To get the angular position along the z-axis, the following equation is required.
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        if self.Init:
            # The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            # print "Entered self.Init due to True"
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix(
                [[np.cos(self.Init_ang), np.sin(self.Init_ang)], [-np.sin(self.Init_ang), np.cos(self.Init_ang)]])
            self.Init_pos.x = Mrot.item((0, 0)) * position.x + Mrot.item((0, 1)) * position.y
            self.Init_pos.y = Mrot.item((1, 0)) * position.x + Mrot.item((1, 1)) * position.y
            self.Init_pos.z = position.z

        Mrot = np.matrix(
            [[np.cos(self.Init_ang), np.sin(self.Init_ang)], [-np.sin(self.Init_ang), np.cos(self.Init_ang)]])

        # We subtract the initial values
        self.globalPos.x = Mrot.item((0, 0)) * position.x + Mrot.item((0, 1)) * position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1, 0)) * position.x + Mrot.item((1, 1)) * position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang


def main(args):
    '''Initializes and cleanup ros node'''
    print
    "Program Started drive_wheels"
    rospy.init_node('goToGoal', anonymous=True)
    ic = goToGoal()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print
        "Shutting down OS chase_object node"
    # cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

