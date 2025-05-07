#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import tf
from apriltag_ros.msg import AprilTagDetectionArray
# from ekf import KalmanFilter
import numpy as np
import statistics

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

class Subscriber():
    def __init__(self):
        tag_detections_up = "/tag_detections_up"
        tag_detections_down = "/tag_detections_down"
        self.sub_info_marker = rospy.Subscriber(tag_detections_up, AprilTagDetectionArray, self.cbGetMarker_up, queue_size = 1)
        self.sub_info_marker = rospy.Subscriber(tag_detections_down, AprilTagDetectionArray, self.cbGetMarker_down, queue_size = 1)
        # self.ekf_theta = KalmanFilter()
        self.init_parame()

    def init_parame(self):
        self.updown = False
        self.marker_2d_pose_x = 0.0
        self.marker_2d_pose_y = 0.0
        self.marker_2d_theta = 0.0
        
        self.marker_2d_theta_list = []
        self.marker_2d_theta_clean_list = []
        # self.ekf_theta.init(1,1,5)

    def SpinOnce(self):
        return self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta

    def cbGetMarker_up(self, msg):
        try:
            if self.updown == True:
                marker_msg = msg.detections[0].pose.pose.pose
                quaternion = (marker_msg.orientation.x, marker_msg.orientation.y, marker_msg.orientation.z, marker_msg.orientation.w)
                theta = tf.transformations.euler_from_quaternion(quaternion)[1]
                # theta = self.ekf_theta.update(theta)
                self.marker_2d_pose_x = -marker_msg.position.z
                self.marker_2d_pose_y = marker_msg.position.x
                self.marker_2d_theta = -theta
                
                if(self.marker_2d_theta != 0.0):
                    self.marker_2d_theta_list.append(self.marker_2d_theta)
                    
            else:
                pass
        except:
            pass

    def cbGetMarker_down(self, msg):
        try:
            if self.updown == False:
                marker_msg = msg.detections[0].pose.pose.pose
                quaternion = (marker_msg.orientation.x, marker_msg.orientation.y, marker_msg.orientation.z, marker_msg.orientation.w)
                theta = tf.transformations.euler_from_quaternion(quaternion)[1]
                # theta = self.ekf_theta.update(theta)
                self.marker_2d_pose_x = -marker_msg.position.z
                self.marker_2d_pose_y = marker_msg.position.x
                self.marker_2d_theta = -theta
                
            else:
                pass
        except:
            pass
    
    def SetMarker2DTheta(self, time):
        initial_time = rospy.Time.now().secs
        while(abs(initial_time - rospy.Time.now().secs) < time):
            print(initial_time - rospy.Time.now().secs)
            self.marker_2d_theta_list.append(self.marker_2d_theta)
        
        threshold = 1
        mean = statistics.mean(self.marker_2d_theta_list)
        stdev = statistics.stdev(self.marker_2d_theta_list)
        upcutoff = mean + threshold * stdev
        downcutoff = mean - threshold * stdev
        clean_list = []
        for i in self.marker_2d_theta_list:
            if(i > downcutoff and i < upcutoff):
               clean_list.append(i)
                
        print(statistics.mean(clean_list))  
        # self.marker_2d_theta_clean_list.clear()
        
        
class main():
    def __init__(self, name):
        self.subscriber = Subscriber()
        print(self.subscriber.SetMarker2DTheta(4))



if __name__ == '__main__':
    rospy.init_node('PBVS_server')
    rospy.logwarn(rospy.get_name() + 'start')
    main = main(rospy.get_name())
    rospy.spin()
