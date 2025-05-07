#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
from enum import Enum
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_msgs.msg import Float32
# from ar_track_alvar_msgs.msg import AlvarMarkers
from apriltag_ros.msg import AprilTagDetectionArray
# from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
# from ekf import KalmanFilter
import Tkinter as tk
from std_msgs.msg import String




class AutomaticParkingVision():
    def __init__(self):
        self.sub_odom_robot = rospy.Subscriber('command', String, self.cbGetCommand, queue_size = 100)
        self.pub_wait = rospy.Publisher('wait', Bool, queue_size=100)
        self.sub_odom_robot = rospy.Subscriber('/odom', Odometry, self.cbGetRobotOdom, queue_size = 1)
        self.sub_info_fixedmarker = rospy.Subscriber('/fixed', PoseWithCovarianceStamped, self.cbGetFixedMarkerOdom, queue_size = 1)
#############
        self.sub_info_marker = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.cbGetMarkerOdom_up, queue_size = 1) 
        # self.sub_info_marker = rospy.Subscriber('/ar_pose_marker_2', AlvarMarkers, self.cbGetMarkerOdom_down, queue_size = 1)
        self.pub_fixed_up = rospy.Publisher('/trigger_up', Bool, queue_size=1)
        self.pub_fixed_down = rospy.Publisher('/trigger_down', Bool, queue_size=1)        
#############
        # self.sub_info_marker = rospy.Subscriber('/visualization_marker_2', Marker, self.cbGetMarkertf, queue_size = 1)
        self.startflag = rospy.Subscriber('/lets_move_finished', Bool, self.cbGetStartflag, queue_size = 1)
        self.fork_psition = rospy.Subscriber('/fork_position', Float32, self.cbGetfork, queue_size = 1)
        
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)#/cmd_vel_mux/input/teleop
        self.ParkingSequence = Enum('ParkingSequence', 'searching_parking_lot changing_direction1 decide1 decide2 decide3 moving_nearby_parking_lot parking_1 changing_direction2 changing_direction3 changing_direction_sec parking_2 dead_reckoning stop finished  back  \
                                     setfork  forkdown_all wait next_station')
        self.NearbySequence = Enum('NearbySequence', 'initial_turn go_straight turn_right parking ')
        
        self.check_wait_time =0
        self.init_pose_and_parame()

        self.front = False
        self.start_flag = False
        self.fixed = False
        self.use_fixed = False
        self.set_camera = True
        self.MARKER_ID_DETECTION = 7  

        self.now_time = 0.0
        self.init_time = 0.0
        self.desire_fork = 0.0
        self.fork_now = 0.0 
        self.initial_robot_pose_x = 0.0 
        self.dead_reckoning_marker = 0.0
        self.go_back = 0.0    
        


        self.parking_step =1
        self.loop_rate = rospy.Rate(20) # 10hz
        self.windows()
        # while not rospy.is_shutdown():
        #     if self.is_odom_received is True and self.start_flag is True:
        #         self.fnParking()
         
        #     self.loop_rate.sleep()

        # rospy.on_shutdown(self.fnShutDown)


    def init_pose_and_parame(self):
        self.current_nearby_sequence = self.NearbySequence.initial_turn.value
        self.current_parking_sequence = self.ParkingSequence.searching_parking_lot.value

        self.robot_2d_pose_x = .0
        self.robot_2d_pose_y = .0
        self.robot_2d_theta = .0
        self.marker_2d_pose_x = .0
        self.marker_2d_pose_y = .0
        self.marker_2d_theta = .0
        self.fork_pose = .0
        ##
        self.parking_dist = .0
        self.back_dist = .0
        self.next_station_back = False
        ##

        self.previous_robot_2d_theta = .0
        self.total_robot_2d_theta = .0
        self.is_triggered = False
        self.is_sequence_finished = False
        self.is_odom_received = False
        self.is_marker_pose_received = False
        self.initial_marker_pose_x = .0
        
        # ekf_theta.init(1,1,5)
        # ekf_x.init(1,1,5)
        # ekf_y.init(1,1,5)


    def cbGetCommand(self, msg):
        exec(msg.data)
    def cbGetfork(self, fork_position_msg):
        self.fork_pose = fork_position_msg.data #used to bo true
        # print("fork_position ", self.fork_pose)

    def cbGetStartflag(self, startflag_msg):
        self.start_flag = startflag_msg.data #used to bo true
    
    def cbGetfixedflag(self, fixedflag_msg):
        if fixedflag_msg == True:
            self.fixed = True
        else :
            self.fixed = False    

    def cbGetRobotOdom(self, wheel_odom_msg):
        if self.is_odom_received == False:
            self.is_odom_received = True 
        
        pos_x, pos_y, theta = self.fnGet2DRobotPose(wheel_odom_msg)

        self.robot_2d_pose_x = pos_x
        self.robot_2d_pose_y = pos_y
        self.robot_2d_theta = theta

        if (self.robot_2d_theta - self.previous_robot_2d_theta) > 5.:
            d_theta = (self.robot_2d_theta - self.previous_robot_2d_theta) - 2 * math.pi
        elif (self.robot_2d_theta - self.previous_robot_2d_theta) < -5.:
            d_theta = (self.robot_2d_theta - self.previous_robot_2d_theta) + 2 * math.pi
        else:
            d_theta = (self.robot_2d_theta - self.previous_robot_2d_theta)

        self.total_robot_2d_theta = self.total_robot_2d_theta + d_theta
        self.previous_robot_2d_theta = self.robot_2d_theta

        self.robot_2d_theta = self.total_robot_2d_theta
    def cbGetMarkerOdom_up(self, msg):
        try:
            if self.is_marker_pose_received == False:
                self.is_marker_pose_received = True
            marker_msg = msg.detections[0].pose.pose.pose
            quaternion = (marker_msg.orientation.x, marker_msg.orientation.y, marker_msg.orientation.z, marker_msg.orientation.w)
            theta = tf.transformations.euler_from_quaternion(quaternion)[1]
            self.marker_2d_pose_x = -marker_msg.position.z
            self.marker_2d_pose_y = marker_msg.position.x
            self.marker_2d_theta = -theta
        except:
            pass

    def cbGetMarkerOdom_down(self, markers_odom_msg):
            if self.fixed==False:
                if self.set_camera == False :  
                    for marker_odom_msg in markers_odom_msg.markers:
                        if marker_odom_msg.id == 7:
                            if self.is_marker_pose_received == False:
                                self.is_marker_pose_received = True
                                
                            pos_x, pos_y, theta = self.fnGet2DMarkerPose(marker_odom_msg)

                            self.marker_2d_pose_x = pos_x
                            self.marker_2d_pose_y = pos_y
                            self.marker_2d_theta = theta - math.pi

                #print math.degrees(self.marker_2d_theta)
    # def cbGetMarkertf(self, markers_tf_msg):
    #     for marker_tf_msg in markers_tf_msg.markers:
    #         print marker_tf_msg.pose.orientation.x
    def cbGetFixedMarkerOdom(self, markers_odom_msg):
        if self.fixed==True:          
            if self.is_marker_pose_received == False:
                self.is_marker_pose_received = True
                
            pos_x, pos_y, theta = self.fnGet2DMarkerPose(markers_odom_msg)

            self.marker_2d_pose_x = pos_x
            self.marker_2d_pose_y = pos_y
            self.marker_2d_theta = theta - math.pi            
########################################################################################################
    def fnParking(self):
        if self.current_parking_sequence == self.ParkingSequence.searching_parking_lot.value:
            self.is_sequence_finished = self.fnSeqSearchingGoal()
            
            if self.is_sequence_finished == True:
                #print "Finished 1"
                self.current_parking_sequence = self.ParkingSequence.changing_direction1.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.changing_direction1.value:
            #print "changing_direction"
            self.is_sequence_finished = self.fnSeqChangingDirection()
            
            if self.is_sequence_finished == True:
                #print "Finished 2"
                self.current_parking_sequence = self.ParkingSequence.decide1.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.decide1.value:
            #print "decide"
            self.is_sequence_finished = self.fnSeqdecide()
            if self.is_sequence_finished == True:
                #print "Finished 3"
                self.current_parking_sequence = self.ParkingSequence.parking_1.value
                #self.current_parking_sequence = self.ParkingSequence.again.value
                self.is_sequence_finished = False
                if self.use_fixed == True:
                    self.fixed = False
                    if self.set_camera == True:
                        self.pub_fixed_up.publish(False) 
                    else:
                        self.pub_fixed_down.publish(False)                   
            else:
                #print "Finished 3"
                if self.use_fixed == True:
                    self.fixed = True
                    if self.set_camera == True:
                        self.pub_fixed_up.publish(True) 
                    else:
                        self.pub_fixed_down.publish(True)

                self.current_parking_sequence = self.ParkingSequence.back.value
                #self.current_parking_sequence = self.ParkingSequence.again.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.back.value:
            self.is_sequence_finished = self.fnSeqBack()
            
            if self.is_sequence_finished == True:
                #print "go to step 1"
                self.init_pose_and_parame()
                self.current_parking_sequence = self.ParkingSequence.changing_direction2.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.changing_direction2.value:
            #print "changing_direction"
            self.is_sequence_finished = self.fnSeqChangingDirection()
            
            if self.is_sequence_finished == True:
                #print "Finished 2"
                self.current_parking_sequence = self.ParkingSequence.decide2.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.decide2.value:
            #print "decide"
            self.decide = 2
            self.is_sequence_finished = self.fnSeqdecide()
            if self.is_sequence_finished == True:
                #print "Finished 3"
                self.current_parking_sequence = self.ParkingSequence.parking_1.value
                #self.current_parking_sequence = self.ParkingSequence.again.value
                self.is_sequence_finished = False
         
            else:
                self.current_parking_sequence = self.ParkingSequence.moving_nearby_parking_lot.value
                #self.current_parking_sequence = self.ParkingSequence.again.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.moving_nearby_parking_lot.value:
            #print "moving_nearby_parking_lot"
            self.is_sequence_finished = self.fnSeqMovingNearbyParkingLot()
            if self.is_sequence_finished == True:
                #print "Finished 3"
                self.current_parking_sequence = self.ParkingSequence.parking_1.value
                #self.current_parking_sequence = self.ParkingSequence.again.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.parking_1.value:
            self.parking_step =1
            self.is_sequence_finished = self.fnSeqParking()
            
            if self.is_sequence_finished == True:
                #print "Finished 4"
                self.current_parking_sequence = self.ParkingSequence.changing_direction3.value
                self.is_sequence_finished = False
                if self.use_fixed == True:
                    self.fixed = False
                    if self.set_camera == True:
                        self.pub_fixed_up.publish(False) 
                    else:
                        self.pub_fixed_down.publish(False)                                   

        elif self.current_parking_sequence == self.ParkingSequence.changing_direction3.value:
            #print "changing_direction"
            self.is_sequence_finished = self.fnSeqChangingDirection()
            
            if self.is_sequence_finished == True:
                #print "Finished 2"
                self.current_parking_sequence = self.ParkingSequence.decide3.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.decide3.value:
            #print "decide"
            self.is_sequence_finished = self.fnSeqdecide()
            if self.is_sequence_finished == True:
                #print "Finished 3"
                self.current_parking_sequence = self.ParkingSequence.wait.value
                self.pub_wait.publish(True)
                #self.current_parking_sequence = self.ParkingSequence.again.value
                self.is_sequence_finished = False
                            
            else:
                #print "Finished 3"
                if self.use_fixed == True:
                    self.fixed = True
                    if self.set_camera == True:
                        self.pub_fixed_up.publish(True) 
                    else:
                        self.pub_fixed_down.publish(True)

                self.current_parking_sequence = self.ParkingSequence.back.value
                #self.current_parking_sequence = self.ParkingSequence.again.value
                self.is_sequence_finished = False

        # elif self.current_parking_sequence == self.ParkingSequence.changing_direction_sec.value:
        #     #print "changing_direction"
        #     self.is_sequence_finished = self.fnSeqlimitdegree()
            
        #     if self.is_sequence_finished == True:
        #         #print "Finished 2"
        #         self.current_parking_sequence = self.ParkingSequence.wait.value
        #         self.is_sequence_finished = False
        
#######################################################################################################################        
        elif self.current_parking_sequence == self.ParkingSequence.dead_reckoning.value:

            self.is_sequence_finished = self.fnseqdead_reckoning()
            if self.is_sequence_finished == True:
                self.current_parking_sequence = self.ParkingSequence.wait.value
                self.pub_wait.publish(True)
                self.is_sequence_finished = False
                self.next_station_back = True

        elif self.current_parking_sequence == self.ParkingSequence.setfork.value:

            self.is_sequence_finished = self.fnSeqfork()
            if self.is_sequence_finished == True:
                if self.next_station_back == True:
                    self.current_parking_sequence = self.ParkingSequence.next_station.value
                    self.is_sequence_finished = False
                    self.next_station_back = False
                else:
                    self.current_parking_sequence = self.ParkingSequence.wait.value
                    self.pub_wait.publish(True)                   
        
        elif self.current_parking_sequence == self.ParkingSequence.next_station.value:

            self.is_sequence_finished = self.fnBack()
            if self.is_sequence_finished == True:
                self.current_parking_sequence = self.ParkingSequence.wait.value
                self.pub_wait.publish(True)
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.forkdown_all.value:

            self.is_sequence_finished = self.fnSeqdown_all()
            if self.is_sequence_finished == True:
                if self.next_station_back == True:
                    self.current_parking_sequence = self.ParkingSequence.next_station.value
                    self.is_sequence_finished = False
                    self.next_station_back = False
                else:
                    self.current_parking_sequence = self.ParkingSequence.wait.value
                    self.pub_wait.publish(True)  

        elif self.current_parking_sequence == self.ParkingSequence.stop.value:
            self.fnStop()
            # print "STOP!!!!"
            # print self.marker_2d_pose_x,"  ",self.marker_2d_pose_y,"  ",math.degrees(self.marker_2d_theta)
            self.current_parking_sequence = self.ParkingSequence.finished.value
            self.start_flag = False
            rospy.on_shutdown(self.fnShutDown)
        
    def fnSeqSearchingGoal(self):
        # print "fnSeqSearchingGoal"

        if self.is_marker_pose_received is False:
            self.desired_angle_turn = -0.6
            self.fnTurn(self.desired_angle_turn)
        else:

            self.fnStop()
            return True

    def fnSeqChangingDirection(self):
        # print "fnSeqChangingDirection"
        
        desired_angle_turn = -1. *  math.atan2(self.marker_2d_pose_y - 0, self.marker_2d_pose_x - 0)
        ##print desired_angle_turn
        if  not self.front:
            if desired_angle_turn <0:
                desired_angle_turn = desired_angle_turn + math.pi
            else:
                desired_angle_turn = desired_angle_turn - math.pi
            

        # rospy.loginfo("desired_angle_turn %f self.marker_2d_pose_x %f self.marker_2d_pose_y %f"
        #  , desired_angle_turn, self.marker_2d_pose_x, self.marker_2d_pose_y)

        self.fnTurn(desired_angle_turn)
        
        if abs(desired_angle_turn) < 0.02  :
            self.fnStop()
            if self.check_wait_time > 10 :
                self.check_wait_time = 0
                return True
            else:
                self.check_wait_time =self.check_wait_time  +1

        elif abs(desired_angle_turn) < 0.035 and self.check_wait_time :
            self.fnStop()
            if self.check_wait_time > 10 :  
                self.check_wait_time = 0
                return True
            else :
                self.check_wait_time =self.check_wait_time  +1
        else:
            self.check_wait_time =0
            return False

    def fnSeqlimitdegree(self):
        self.loop_rate = rospy.Rate(30)
        # print "fnSeqlimitdegree"
        if self.front:
            desired_angle_turn = np.sign(self.marker_2d_theta)*0.12
        else:
            desired_angle_turn = -np.sign(self.marker_2d_theta)*0.12
        
        #print desired_angle_turn
        self.fnTurn(desired_angle_turn)

        #print math.degrees(self.marker_2d_theta)ffnde
        
        
        
        
        if self.front:
            
            a=(180.0-abs(math.degrees(self.marker_2d_theta)))
            # print (a)
         
            if (a) < 0.8:
                self.fnStop()
                if self.check_wait_time > 10:
                    self.check_wait_time = 0
                    return True
                else:
                    self.check_wait_time =self.check_wait_time  +1
            elif (a) < 1.2 and self.check_wait_time:
                self.fnStop()
                if self.check_wait_time > 10:
                    self.check_wait_time = 0
                    return True
                else:
                    self.check_wait_time =self.check_wait_time  +1
            else:
                self.check_wait_time =0
                return False
            
        else:
            # print abs(math.degrees(self.marker_2d_theta))
            if  abs(math.degrees(self.marker_2d_theta)) < 0.5:
                self.fnStop()
                if self.check_wait_time> 8:
                    self.check_wait_time = 0
                    return True
                else:
                    self.check_wait_time =self.check_wait_time  +1
            elif  abs(math.degrees(self.marker_2d_theta)) < 0.8 and self.check_wait_time:
                self.fnStop()
                if self.check_wait_time > 8:
                    self.check_wait_time = 0
                    return True
                else:
                    self.check_wait_time =self.check_wait_time  +1
            else:
                self.check_wait_time =0
                return False

                


    def fnSeqMovingNearbyParkingLot(self):
        if self.current_nearby_sequence == self.NearbySequence.initial_turn.value:
            if self.is_triggered == False:
                self.is_triggered = True
                self.initial_robot_pose_theta = self.robot_2d_theta
                self.initial_robot_pose_x = self.robot_2d_pose_x
                self.initial_robot_pose_y = self.robot_2d_pose_y

                self.initial_marker_pose_theta = self.marker_2d_theta
                self.initial_marker_pose_x = self.marker_2d_pose_x

            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = (math.pi / 2.0) + self.initial_marker_pose_theta - (self.robot_2d_theta - self.initial_robot_pose_theta)
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = -(math.pi / 2.0) + self.initial_marker_pose_theta - (self.robot_2d_theta - self.initial_robot_pose_theta)

            # rospy.loginfo("desired_angle_turn %f self.initial_marker_pose_theta %f self.robot_2d_theta %f self.initial_robot_pose_theta %f"
            # , desired_angle_turn, self.initial_marker_pose_theta, self.robot_2d_theta, self.initial_robot_pose_theta)

            desired_angle_turn = -1. * desired_angle_turn
            # print desired_angle_turn
            self.fnTurn(desired_angle_turn)

            if abs(desired_angle_turn) < 0.03:
                self.fnStop()
                if self.check_wait_time >10:
                    self.check_wait_time = 0
                    self.current_nearby_sequence = self.NearbySequence.go_straight.value
                    self.is_triggered = False
                else:
                    self.check_wait_time =self.check_wait_time +1
            elif abs(desired_angle_turn) < 0.045 and self.check_wait_time :
                self.fnStop()
                if self.check_wait_time > 10:
                    self.check_wait_time = 0
                    self.current_nearby_sequence = self.NearbySequence.go_straight.value
                    self.is_triggered = False
                else:
                    self.check_wait_time =self.check_wait_time +1
            else:
                self.check_wait_time =0    
            # rospy.loginfo("fnSeqMovingNearbyParkingLot 1")

        elif self.current_nearby_sequence == self.NearbySequence.go_straight.value:
            if self.is_triggered == False:
                self.is_triggered = True
                self.initial_robot_pose_x = self.robot_2d_pose_x
                self.initial_robot_pose_y = self.robot_2d_pose_y

            dist_from_start = self.fnCalcDistPoints(self.initial_robot_pose_x, self.robot_2d_pose_x, self.initial_robot_pose_y, self.robot_2d_pose_y)
            if self.front:
                desired_dist = (self.initial_marker_pose_x) * abs(math.cos((math.pi / 2.) - self.initial_marker_pose_theta))
            else:
                # print(self.initial_marker_pose_theta)
                # print(self.initial_marker_pose_x)
                desired_dist = -1* self.initial_marker_pose_x * abs(math.cos((math.pi / 2.) - self.initial_marker_pose_theta))
                # print(desired_dist)
                # a=input()
            
            #if self.front:
            remained_dist = desired_dist - dist_from_start 
            if remained_dist < 0  :remained_dist =0
            #else:
            #    remained_dist = dist_from_start - desired_dist 
            
            #remained_dist = desired_dist - dist_from_start
            # rospy.loginfo("remained_dist %f desired_dist %f dist_from_start %f", remained_dist, desired_dist, dist_from_start)
            #self.fnStop()
            
            self.fnGoStraight(desired_dist)

            if abs(remained_dist) < 0.01:
                self.fnStop()
                # if self.check_wait_time > 5:
                #     self.check_wait_time = 0
                self.current_nearby_sequence = self.NearbySequence.turn_right.value
                self.is_triggered = False
                # else:
                    # self.check_wait_time =self.check_wait_time +1
            # elif  abs(remained_dist) < 0.02 and self.check_wait_time:
            #     self.fnStop()
            #     if self.check_wait_time > 5:
            #         self.check_wait_time = 0
            #         self.current_nearby_sequence = self.NearbySequence.turn_right.value
            #     else:
            #         self.check_wait_time =self.check_wait_time +1
            # else:
            #     self.check_wait_time =0      
            # rospy.loginfo("fnSeqMovingNearbyParkingLot 2")

        elif self.current_nearby_sequence == self.NearbySequence.turn_right.value:
            if self.is_triggered == False:
                self.is_triggered = True
                self.initial_robot_pose_theta = self.robot_2d_theta
            if self.front:
                if self.initial_marker_pose_theta < 0.0:
                    desired_angle_turn = -(math.pi / 2.0) + (self.robot_2d_theta - self.initial_robot_pose_theta)
                elif self.initial_marker_pose_theta > 0.0:
                    desired_angle_turn = (math.pi / 2.0) + (self.robot_2d_theta - self.initial_robot_pose_theta)
            else:
                if self.initial_marker_pose_theta < 0.0:
                    desired_angle_turn = (math.pi / 2.0) + (self.robot_2d_theta - self.initial_robot_pose_theta)
                elif self.initial_marker_pose_theta > 0.0:
                    desired_angle_turn = -(math.pi / 2.0) + (self.robot_2d_theta - self.initial_robot_pose_theta)
            
            # rospy.loginfo("desired_angle_turn %f self.robot_2d_theta %f self.initial_robot_pose_theta %f"
            # , desired_angle_turn, self.robot_2d_theta, self.initial_robot_pose_theta)
            # print desired_angle_turn
            self.fnTurn(desired_angle_turn)
            # rospy.loginfo("fnSeqMovingNearbyParkingLot 3")
            if abs(desired_angle_turn) < 0.03:
                self.fnStop()
                if self.check_wait_time > 20:
                    self.check_wait_time = 0
                    self.current_nearby_sequence = self.NearbySequence.parking.value
                    self.is_triggered = False
                    return True                
                else:
                    self.check_wait_time =self.check_wait_time  +1
            elif abs(desired_angle_turn) < 0.045 and self.check_wait_time:
                self.fnStop()
                if self.check_wait_time > 20:
                    self.check_wait_time = 0
                    self.current_nearby_sequence = self.NearbySequence.parking.value
                    self.is_triggered = False
                    return True                
                else:
                    self.check_wait_time =self.check_wait_time  +1
            else:
                self.check_wait_time =0    
        return False
    def fnSeqdecide(self):
        self.dist = abs(self.marker_2d_pose_x*math.cos((np.pi/2)-abs(self.marker_2d_theta)))
        # rospy.loginfo("fnSeqdecide")
        # print self.dist
        # print self.check_wait_time
        if self.set_camera == True:
            self.decide_dist = 0.03 #偏離多少公分要後退
        else:
            self.decide_dist = 0.03   
        
        if  self.dist < self.decide_dist:
            #print "again"
            return True
        else:
            return False    
    # def fnSeqifagain(self):
    #     while self.fnSeqChangingDirection() == False:
            
    #         print "wait"
    #     rospy.loginfo("fnSeqifagain")
    #     #print self.marker_2d_pose_x,"  ",self.marker_2d_pose_y,"  ",math.degrees(self.marker_2d_theta)
    #     self.dist = abs(self.marker_2d_pose_x*math.cos((np.pi/2)-abs(self.marker_2d_theta)))
    #     print self.dist
    #     print self.check_wait_time 
    #     if  self.dist < 0.02:
    #         #print "again"
    #         return True
    #     else:
    #         return False


    def fnSeqParking(self):
        desired_angle_turn = math.atan2(self.marker_2d_pose_y - 0, self.marker_2d_pose_x - 0)
        #desired_angle_turn = math.atan2(self.marker_2d_pose_y - self.robot_2d_pose_y, self.marker_2d_pose_x - self.robot_2d_pose_x)
        
        # rospy.loginfo("fnSeqParking")
        # print self.robot_2d_pose_x ,self.robot_2d_pose_y
        if  not self.front:
            if desired_angle_turn <0:
                desired_angle_turn = desired_angle_turn + math.pi
            else:
                desired_angle_turn = desired_angle_turn - math.pi
        # if not self.front:
        #     desired_angle_turn = -desired_angle_turn 
        # print self.marker_2d_theta
        # print desired_angle_turn
        if self.set_camera == True:
            self.parking_dist = 0.3 #0.55
        else :
            self.parking_dist =  0.4
        # if self.parking_step is 1:
        #     dist_limt =0.4
        # elif self.parking_step is 2:
        #     dist_limt =0.35
        #     desired_angle_turn=0
        # if abs(desired_angle_turn) < 0.15:
        #     desired_angle_turn=0
        
        self.fnTrackMarker(-desired_angle_turn)
        #self.fnTrackMarker(0)
        # print "x"

        # print self.marker_2d_pose_x
        
        # print self.parking_step
        
        if (abs(self.marker_2d_pose_x) < self.parking_dist)  :
            self.fnStop()
            if self.check_wait_time > 10:
                self.check_wait_time = 0
                return True
            else:
                self.check_wait_time =self.check_wait_time  +1
        elif (abs(self.marker_2d_pose_x) < self.parking_dist) and self.check_wait_time:
            self.fnStop()
            if self.check_wait_time > 10:
                self.check_wait_time = 0
                return True
            else:
                self.check_wait_time =self.check_wait_time  +1
        else:
            self.check_wait_time =0
            return False
    # def fnSeqParking2(self):


    def fnSeqBack(self):
        if self.set_camera == True:
            self.back_dist = 0.6
        else:
            self.back_dist = 0.55   
        self.fnGoBack()
        if abs(self.marker_2d_pose_x) > self.back_dist:
            self.fnStop()
            return True
        else:
            return False
    def fnseqdead_reckoning(self):
        if self.is_triggered == False:
            self.is_triggered = True
            self.initial_robot_pose_x = self.robot_2d_pose_x
            self.initial_marker_pose_x = self.marker_2d_pose_x 
        self.dist = abs(self.marker_2d_pose_x - self.initial_marker_pose_x)
        if self.set_camera == True:
            self.dead_reckoning_dist = 0.25
        else:
            self.dead_reckoning_dist = 0.1
        
        # print("self.dist < self.dead_reckoning_dist-0.01:  ",self.dist < self.dead_reckoning_dist-0.01)
        if self.dist > self.dead_reckoning_dist:
            self.fnStop()
            self.is_triggered = False
            return True
        else:
            self.fnGoStraight(self.dist)
            return False

          
    def fnSeqfork(self):
        # if self.is_triggered == False:
        #     self.is_triggered = True
        #     t1 = rospy.get_rostime()
        #     self.init_time = (t1.secs) +(t1.nsecs/math.pow(10,9))
        # t2 = rospy.get_rostime()
        # self.now_time =  (t2.secs) +(t2.nsecs/math.pow(10,9))
        # # t1_initial=rospy.Time(self.init_time.secs,self.init_time.nsecs)
        # # t2_now = rospy.Time(self.now_time.secs,self.now_time.nsecs)
        # print("self.desire_fork = ",self.desire_fork ,"self.fork_now = ",self.fork_now)
        # print('!!!!!!',self.now_time-self.init_time)
        # up_or_down = math.copysign(1,(self.desire_fork - self.fork_now))
        # self.fnfork(up_or_down)
        # if (self.now_time - self.init_time) > abs(self.desire_fork - self.fork_now):
        #     self.fnStop()
        #     self.fork_now = self.desire_fork
        #     self.is_triggered = False
        #     return True
        # else:
        #     return False
        if(self.fork_pose < self.desire_fork-0.01):
            # print("fork_pose ", self.fork_pose)
            # print("desire fork ", self.desire_fork)
            self.fnfork(1)
            return False
        elif(self.fork_pose > self.desire_fork+0.01):
            self.fnfork(-1)
            return False
        else:
            self.fnStop()
            return True


    def fnSeqdown(self):
        if self.fork_pose > 0.0:
            self.fndown()
            return False
        elif self.fork_pose == 0.0:
            self.fnStop()
            return True

    def fnSeqdown_all(self):
        self.fnfork(-1)
        if self.fork_pose == 0.0:
            self.fnStop()
            self.fork_now = 0.0 
            return True

    def fnBack(self):
        self.fnGoBack()
        if self.is_triggered == False:
            self.is_triggered = True
            self.initial_marker_pose_x = self.marker_2d_pose_x 
        dist =(self.initial_marker_pose_x - self.marker_2d_pose_x)
        self.go_back = 0.3
        # print(self.go_back)
        if dist > self.go_back :#wanna close num be bigger
            self.fnStop()
            self.is_triggered = False
            return True
        else:
            return False

    def fnStop(self):
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        #self.pub_cmd_vel.publish(twist)
        self.cmd_pub(twist)

    def fnTurn(self, theta):
        Kp = 1.0 #1.0
 
        angular_z = Kp * theta

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        
        # if angular_z< 0.2 and angular_z> -0.2:
        #     if angular_z > 0:
        #         angular_z =0.1
        #     else:
        #         angular_z =-0.1
        twist.angular.z = -angular_z
        #self.pub_cmd_vel.publish(twist)
        self.cmd_pub(twist)

    def fnGoStraight(self,v):
        twist = Twist()
        twist.linear.x = v*0.8
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        # print(twist.linear.x)
        #self.pub_cmd_vel.publish(twist)
        self.cmd_pub(twist)

    def fnGoBack(self):
        twist = Twist()
        twist.linear.x = -0.12
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        #self.pub_cmd_vel.publish(twist)
        self.cmd_pub(twist)

    def fnfork(self,direction):
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = direction
        twist.angular.z = 0
        #self.pub_cmd_vel.publish(twist)
        self.cmd_pub(twist)

    # def fndown(self):
    #     twist = Twist()
    #     twist.linear.x = 0
    #     twist.linear.y = 0
    #     twist.linear.z = 0
    #     twist.angular.x = 0
    #     twist.angular.y = -1
    #     twist.angular.z = 0
    #     #self.pub_cmd_vel.publish(twist)
    #     self.cmd_pub(twist)

    def fnTrackMarker(self, theta):
        Kp = 0.8

        angular_z = Kp * theta

        twist = Twist()
        twist.linear.x = 0.1
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0

        #if angular_z< 0.2 and angular_z> -0.2:
        #    if angular_z > 0:
        #        angular_z =0.2
        #    else:
        #        angular_z =-0.2

        twist.angular.z = -angular_z *0.15
        #self.pub_cmd_vel.publish(twist)
        self.cmd_pub(twist)

    def cmd_pub(self, twist):
        if not self.front:
            twist.linear.x = -twist.linear.x
        #    twist.angular.z = -twist.angular.z
        if twist.angular.z > 0.8:
            twist.angular.z =0.8 
        elif twist.angular.z < -0.8:
            twist.angular.z =-0.8 
        if twist.linear.x > 0 and twist.linear.x < 0.01:
            twist.linear.x =0.01
        elif twist.linear.x < 0 and twist.linear.x > -0.01:
            twist.linear.x =-0.01   

        if twist.linear.x > 0 and twist.linear.x > 0.1:
            twist.linear.x =0.08
        elif twist.linear.x < 0 and twist.linear.x > -0.1:
            twist.linear.x =-0.08                     
        if twist.angular.z > 0 and twist.angular.z < 0.1:
            twist.angular.z =0.1
        elif twist.angular.z < 0 and twist.angular.z > -0.1:
            twist.angular.z =-0.1
        self.pub_cmd_vel.publish(twist)
        


    def fnGet2DRobotPose(self, wheel_odom_msg):
        quaternion = (wheel_odom_msg.pose.pose.orientation.x, wheel_odom_msg.pose.pose.orientation.y, wheel_odom_msg.pose.pose.orientation.z, wheel_odom_msg.pose.pose.orientation.w)
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]

        if theta < 0:
            theta = theta + np.pi * 2
        if theta > np.pi * 2:
            theta = theta - np.pi * 2

        pos_x = wheel_odom_msg.pose.pose.position.x
        pos_y = wheel_odom_msg.pose.pose.position.y

        return pos_x, pos_y, theta

    def fnGet2DMarkerPose(self, marker_odom_msg):
        quaternion = (marker_odom_msg.pose.pose.orientation.x, marker_odom_msg.pose.pose.orientation.y, marker_odom_msg.pose.pose.orientation.z, marker_odom_msg.pose.pose.orientation.w)
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]

        theta = theta + np.pi / 2.
        #rospy.loginfo("\ntheta : %4f ,%4f", theta,math.degrees(theta))

        if theta < 0:       #normolize
            theta = theta + np.pi * 2
        if theta > np.pi * 2:
            theta = theta - np.pi * 2

        pos_x = marker_odom_msg.pose.pose.position.x
        pos_y = marker_odom_msg.pose.pose.position.y
	    
        # theta = ekf_theta.update(theta)
        # pos_x = ekf_x.update(pos_x)
        # pos_y = ekf_y.update(pos_y)
        # print ekf_x.update(pos_x),ekf_y.update(pos_y),ekf_theta.update(theta) ,"\n"
        return pos_x, pos_y, theta

    def fnCalcDistPoints(self, x1, x2, y1, y2):
        return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

    def fnShutDown(self):
        # rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        ##self.pub_cmd_vel.publish(twist)


    def main(self):
        rospy.spin()
    def start(self):
        self.start_flag = not self.start_flag
        # self.current_nearby_sequence = self.NearbySequence.initial_turn.value
        # self.current_parking_sequence = self.ParkingSequence.searching_parking_lot.value
        self.init_pose_and_parame()
    def dead(self):
        self.current_parking_sequence = self.ParkingSequence.dead_reckoning.value
    # def fork(self):
    #     self.current_parking_sequence = self.ParkingSequence.setfork.value
    def ALL_Down(self):
        self.current_parking_sequence = self.ParkingSequence.forkdown_all.value
    def forklayer_1(self):
        self.current_parking_sequence = self.ParkingSequence.setfork.value
        self.desire_fork = 0.0
    def forklayer_1_1(self):
        self.current_parking_sequence = self.ParkingSequence.setfork.value
        self.desire_fork = 0.12
    def forklayer_2(self):
        self.current_parking_sequence = self.ParkingSequence.setfork.value
        self.desire_fork = 0.43
    def forklayer_2_2(self):
        self.current_parking_sequence = self.ParkingSequence.setfork.value
        self.desire_fork = 0.51
    def forklayer_3(self):
        self.current_parking_sequence = self.ParkingSequence.setfork.value
        self.desire_fork = 16.0
    def forklayer_3_3(self):
        self.current_parking_sequence = self.ParkingSequence.setfork.value
        self.desire_fork = 18.0
    def cam(self):
        self.set_camera = not self.set_camera 
        if self.set_camera == True:
            self.MARKER_ID_DETECTION = 2
        else:
            self.MARKER_ID_DETECTION = 7    

    def windows(self):
        self.window = tk.Tk()
        self.window.geometry('450x450+1700+560') 
        button_base = 35
        self.cambutton = tk.Button(self.window, text = "Camera", command = self.cam)
        self.cambutton.place(x = 0, y = 0)
        self.start_button = tk.Button(self.window, text = "Start", command = self.start)
        self.start_button.place(x = 80, y = 0)
        self.deadbutton = tk.Button(self.window, text = "Dead reckoning", command = self.dead)
        self.deadbutton.place(x = 140, y = 0)
        # self.forkdownbutton = tk.Button(self.window, text = "set_fork", command = self.fork)
        # self.forkdownbutton.place(x = 140, y = 0)
        self.forkdownbutton = tk.Button(self.window, text = "ALL_Down", command = self.ALL_Down)
        self.forkdownbutton.place(x = 280, y = 0)

        self.forkbutton1 = tk.Button(self.window, text = "Layer 1", command = self.forklayer_1)
        self.forkbutton1.place(x = 0, y = button_base+30)
        self.forkbutton1_1 = tk.Button(self.window, text = "Layer 1-1", command = self.forklayer_1_1)
        self.forkbutton1_1.place(x = 0, y = button_base)
        self.forkbutton2 = tk.Button(self.window, text = "Layer 2", command = self.forklayer_2)
        self.forkbutton2.place(x = 100, y = button_base+30)
        self.forkbutton2_2 = tk.Button(self.window, text = "Layer 2-2", command = self.forklayer_2_2)
        self.forkbutton2_2.place(x = 100, y = button_base)
        self.forkbutton3 = tk.Button(self.window, text = "Layer 3", command = self.forklayer_3)
        self.forkbutton3.place(x = 200, y = button_base+30)
        self.forkbutton3_3 = tk.Button(self.window, text = "Layer 3-3", command = self.forklayer_3_3)
        self.forkbutton3_3.place(x = 200, y = button_base)

        self.cam_flag = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')
        self.cam_start_flag = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')

        self.buttonstart_flag = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')
        self.button_start_flag = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')
        

        self.labelParkingSequence = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')
        self.label_ParkingSequence = tk.Label(self.window, text="", font=('Helvetica', 12), fg='#19CAAD') 

        self.labelNearbySequence = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')
        self.label_NearbySequence = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black') 

        self.labelrobot_2d_pose_x = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')
        self.label_robot_2d_pose_x = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black') 

        self.labelrobot_2d_pose_y= tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')
        self.label_robot_2d_pose_y = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')  

        self.labelrobot_2d_theta= tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')
        self.label_robot_2d_theta = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')   

        self.labelmarker_2d_pose_x= tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')
        self.label_marker_2d_pose_x = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')        

        self.labelmarker_2d_pose_y= tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')
        self.label_marker_2d_pose_y = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')     

        self.labelmarker_2d_theta= tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')
        self.label_marker_2d_theta = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black') 

        self.labelfork_pose= tk.Label(self.window, text="", font=('Helvetica', 12), fg='black')
        self.label_fork_pose = tk.Label(self.window, text="", font=('Helvetica', 12), fg='black') 
        
        self.update_window()
        self.window.mainloop()

    def update_window(self):

        if self.is_odom_received is True and self.start_flag is True:
                self.fnParking()        
        self.loop_rate.sleep()

        if self.set_camera == True:
            camera = "Up camera"
        else:
            camera = "Down camera"

        if self.current_parking_sequence == self.ParkingSequence.searching_parking_lot.value:
            sequence = "Searching parking location"
        elif self.current_parking_sequence == self.ParkingSequence.changing_direction1.value:
            sequence = "Changing direction1"
        elif self.current_parking_sequence == self.ParkingSequence.decide1.value:
            sequence = "Decide 1"            
        elif self.current_parking_sequence == self.ParkingSequence.back.value:
            sequence = "Back"
        elif self.current_parking_sequence == self.ParkingSequence.changing_direction2.value:
            sequence = "Changing direction2"    
        elif self.current_parking_sequence == self.ParkingSequence.decide2.value:
            sequence = "Decide 2"           
        elif self.current_parking_sequence == self.ParkingSequence.moving_nearby_parking_lot.value:
            sequence = "Moving nearby parking location"
        elif self.current_parking_sequence == self.ParkingSequence.parking_1.value:
            sequence = "Parking 1"
        elif self.current_parking_sequence == self.ParkingSequence.changing_direction3.value:
            sequence = "Changing direction3"    
        elif self.current_parking_sequence == self.ParkingSequence.decide3.value:
            sequence = "Decide3" 
        elif self.current_parking_sequence == self.ParkingSequence.changing_direction_sec.value:
            sequence = "Changing direction second" 
        elif self.current_parking_sequence == self.ParkingSequence.setfork.value:
            sequence = "Set fork"
        elif self.current_parking_sequence == self.ParkingSequence.parking_2.value:
            sequence = "Parking 2"
        elif self.current_parking_sequence == self.ParkingSequence.dead_reckoning.value:
            sequence = "Dead reckoning"
        elif self.current_parking_sequence == self.ParkingSequence.wait.value:
            sequence = "Waits"
        elif self.current_parking_sequence == self.ParkingSequence.next_station.value:
            sequence = "Next station"            
        elif self.current_parking_sequence == self.ParkingSequence.stop.value:
            sequence = "Stop"
        elif self.current_parking_sequence == self.ParkingSequence.finished.value:
            sequence = "Finished"
        elif self.current_parking_sequence == self.ParkingSequence.forkdown_all.value:
            sequence = "Forkdown All"

        base0 = 100
        self.cam_start_flag.configure(text="Camera")
        self.cam_start_flag.place(x=0, y=base0)        
        self.cam_flag.configure(text=camera)
        self.cam_flag.place(x=200, y=base0)
                
        base = base0+30
        self.buttonstart_flag.configure(text="Start_flag ")
        self.buttonstart_flag.place(x=0, y=base)        
        self.button_start_flag.configure(text=self.start_flag)
        self.button_start_flag.place(x=200, y=base)

        self.labelParkingSequence.configure(text="ParkingSequence ")
        self.labelParkingSequence.place(x=0, y=base+30)        
        self.label_ParkingSequence.configure(text=sequence)
        self.label_ParkingSequence.place(x=200, y=base+30)

        base1 = base+70
        self.labelrobot_2d_pose_x.configure(text="Robot 2d Pose x: ")
        self.labelrobot_2d_pose_x.place(x=0, y=base1)        
        self.label_robot_2d_pose_x.configure(text=self.robot_2d_pose_x)
        self.label_robot_2d_pose_x.place(x=200, y=base1)

        self.labelrobot_2d_pose_y.configure(text="Robot 2d Pose y: ")
        self.labelrobot_2d_pose_y.place(x=0, y=base1+30)
        self.label_robot_2d_pose_y.place(x=200, y=base1+30)
        self.label_robot_2d_pose_y.configure(text=self.robot_2d_pose_y)

        self.labelrobot_2d_theta.configure(text="Robot 2d theta: ")
        self.labelrobot_2d_theta.place(x=0, y=base1+60)
        self.label_robot_2d_theta.place(x=200, y=base1+60)
        self.label_robot_2d_theta.configure(text=self.robot_2d_theta)

        base2 = base1+100
        self.labelmarker_2d_pose_x.configure(text="Marker 2d Pose x: ")
        self.labelmarker_2d_pose_x.place(x=0, y=base2)
        self.label_marker_2d_pose_x.place(x=200, y=base2)
        self.label_marker_2d_pose_x.configure(text=self.marker_2d_pose_x)

        self.labelmarker_2d_pose_y.configure(text="Marker 2d Pose y: ")
        self.labelmarker_2d_pose_y.place(x=0, y=base2+30)
        self.label_marker_2d_pose_y.place(x=200, y=base2+30)
        self.label_marker_2d_pose_y.configure(text=self.marker_2d_pose_y)

        self.labelmarker_2d_theta.configure(text="Marker 2d theta: ")
        self.labelmarker_2d_theta.place(x=0, y=base2+60)
        self.label_marker_2d_theta.place(x=200, y=base2+60)
        self.label_marker_2d_theta.configure(text=math.degrees(self.marker_2d_theta))

        base3 = base2+100
        self.labelfork_pose.configure(text="Fork pose: ")
        self.labelfork_pose.place(x=0, y=base3)
        self.label_fork_pose.place(x=200, y=base3)
        self.label_fork_pose.configure(text=self.fork_pose)
        self.window.after(100, self.update_window)


if __name__ == '__main__':
    rospy.init_node('automatic_parking_vision')
    # ekf_theta = KalmanFilter()
    # ekf_x = KalmanFilter()
    # ekf_y = KalmanFilter()    
    node = AutomaticParkingVision()
    node.main()
    

    

