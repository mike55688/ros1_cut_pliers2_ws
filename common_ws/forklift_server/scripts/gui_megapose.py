#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import tf
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import math
from forklift_msg.msg import meteorcar
from ekf import KalmanFilter
import tkinter as tk

class Subscriber():
    def __init__(self):
        odom_topic = rospy.get_param(rospy.get_name() + "/odom_topic", "/odom")
        shelf_topic = rospy.get_param(rospy.get_name() + "/shelf_topic", "/shelf")
        pallet_topic = rospy.get_param(rospy.get_name() + "/pallet_topic", "/pallet")
        forkpos = rospy.get_param(rospy.get_name() + "/forkpos", "/forkpos")
        self.offset_x = rospy.get_param(rospy.get_name() + "/offset_x", 0.0)
        self.sub_info_marker = rospy.Subscriber(shelf_topic, Pose, self.cbGetShelf, queue_size = 1)
        self.sub_info_marker = rospy.Subscriber(pallet_topic, Pose, self.cbGetPallet, queue_size = 1)
        self.sub_odom_robot = rospy.Subscriber(odom_topic, Odometry, self.cbGetRobotOdom, queue_size = 1)
        self.pub_fork = rospy.Publisher('/cmd_fork', meteorcar, queue_size = 1, latch=True)
        self.ekf_theta = KalmanFilter()
        self.init_parame()
        self.windows()

    def init_parame(self):
        # Odometry_param
        self.is_odom_received = False
        self.robot_2d_pose_x = 0.0
        self.robot_2d_pose_y = 0.0
        self.robot_2d_theta = 0.0
        self.previous_robot_2d_theta = 0.0
        self.total_robot_2d_theta = 0.0
        # AprilTag_param
        self.updown = False
        self.shelf_2d_pose_x = 0.0
        self.shelf_2d_pose_y = 0.0
        self.shelf_2d_theta = 0.0
        self.pallet_2d_pose_x = 0.0
        self.pallet_2d_pose_y = 0.0
        self.pallet_2d_theta = 0.0
        # Forklift_param
        self.forwardbackpostion = 0.0
        self.updownposition = 0.0
        #ekf
        self.ekf_theta.init(1,1,5)

    def cbGetPallet(self, msg):
            marker_msg = msg
            quaternion = (marker_msg.orientation.x, marker_msg.orientation.y, marker_msg.orientation.z, marker_msg.orientation.w)
            theta = tf.transformations.euler_from_quaternion(quaternion)[1]
            self.pallet_2d_pose_x = -marker_msg.position.z
            self.pallet_2d_pose_y = marker_msg.position.x + self.offset_x
            self.pallet_2d_theta = -theta
            # rospy.loginfo("Pose: x={:.3f}, y={:.3f}, theta={:.3f}".format(self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta))

    def cbGetShelf(self, msg):
            marker_msg = msg
            quaternion = (marker_msg.orientation.x, marker_msg.orientation.y, marker_msg.orientation.z, marker_msg.orientation.w)
            theta = tf.transformations.euler_from_quaternion(quaternion)[1]
            self.shelf_2d_pose_x = -marker_msg.position.z
            self.shelf_2d_pose_y = marker_msg.position.x + self.offset_x
            self.shelf_2d_theta = -theta
            # rospy.loginfo("Pose: x={:.3f}, y={:.3f}, theta={:.3f}".format(self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta))

    def cbGetRobotOdom(self, msg):
        if self.is_odom_received == False:
            self.is_odom_received = True 

        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        if theta < 0:
            theta = theta + math.pi * 2
        if theta > math.pi * 2:
            theta = theta - math.pi * 2

        self.robot_2d_pose_x = msg.pose.pose.position.x
        self.robot_2d_pose_y = msg.pose.pose.position.y
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

    def cbGetforkpos(self, msg):
        # self.forwardbackpostion = msg.forwardbackpostion
        self.updownposition = msg.fork_position


    def windows(self):
        self.window = tk.Tk()
        self.window.geometry('500x500')
        self.labels = {
            'robot_2d_pose_x': [0, 0],
            'robot_2d_pose_y': [0, 40],
            'robot_2d_theta': [0, 80],
            'pallet_2d_pose_x': [0, 140],
            'pallet_2d_pose_y': [0, 180],
            'pallet_2d_theta': [0, 220],
            'shelf_2d_pose_x': [0, 260],
            'shelf_2d_pose_y': [0, 300],
            'shelf_2d_theta': [0, 340],
            'fork_updown_position': [0, 400],
            # 'fork_forwardback_position': [0, 440]
        }
        for key, value in self.labels.items():
            self.labels[key] = [tk.Label(self.window, text=f"{key.replace('_', ' ').title()}: "), tk.Label(self.window, text="")]
            self.labels[key][0].place(x=value[0], y=value[1])
            self.labels[key][1].place(x=190, y=value[1])
        while not rospy.is_shutdown():
            self.update_window()
            self.window.update()
            rospy.sleep(0.05) # Set the desired update rate
        self.window.destroy()
        # self.update_window()
        # self.window.mainloop()

    def update_window(self):
        update_values = {
            'robot_2d_pose_x': self.robot_2d_pose_x,
            'robot_2d_pose_y': self.robot_2d_pose_y,
            'robot_2d_theta': math.degrees(self.robot_2d_theta),
            'pallet_2d_pose_x': self.pallet_2d_pose_x,
            'pallet_2d_pose_y': self.pallet_2d_pose_y,
            'pallet_2d_theta': math.degrees(self.pallet_2d_theta),
            'shelf_2d_pose_x': self.shelf_2d_pose_x,
            'shelf_2d_pose_y': self.shelf_2d_pose_y,
            'shelf_2d_theta': math.degrees(self.shelf_2d_theta),
            'fork_updown_position': self.updownposition,
            # 'fork_forwardback_position': self.forwardbackpostion
        }
        for key, value in update_values.items():
            self.labels[key][1].configure(text=value)

        # self.window.after(50, self.update_window)

if __name__ == '__main__':
    rospy.init_node('gui')
    subscriber = Subscriber()
    rospy.spin()
