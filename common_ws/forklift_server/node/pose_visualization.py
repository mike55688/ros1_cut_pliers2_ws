#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import tf
import tkinter as tk
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry

class PoseVisualization:
    def __init__(self):
        rospy.init_node('pose_visualization', anonymous=True)
        self.init_parameters()  
        self.get_parameters()   
        self.create_subscribers()  

        self.root = tk.Tk()
        self.root.title("蘋果姿態估測可視化")

        label_font = ("Arial", 20)  

        self.pallet_pose_label = tk.Label(self.root, text="蘋果位置: x=0.0, y=0.0, theta=0.0", font=label_font)
        self.pallet_pose_label.pack()

        self.pallet_z_pose_label = tk.Label(self.root, text="蘋果位置: z=0.0", font=label_font)
        self.pallet_z_pose_label.pack()


        self.update_gui() 
        self.root.mainloop() 

    def __del__(self):
        if not rospy.is_shutdown():
            rospy.signal_shutdown("節點關閉")

    def init_parameters(self):
        self.robot_2d_pose_x = 0.0
        self.robot_2d_pose_y = 0.0
        self.robot_2d_theta = 0.0
        self.previous_robot_2d_theta = 0.0
        self.total_robot_2d_theta = 0.0

        self.marker_2d_pose_x = 0.0
        self.marker_2d_pose_y = 0.0
        self.marker_2d_pose_z = 0.0
        self.marker_2d_theta = 0.0

        self.fruit_2d_pose_x = 0.0
        self.fruit_2d_pose_y = 0.0
        self.fruit_2d_pose_z = 0.0
        self.fruit_2d_theta = 0.0

        # 移除叉車變數
        # self.updownposition = 0.0

    def get_parameters(self):
        # 從參數伺服器獲取主題名稱，參考 PBVS_server_megapose_differential_drive.py
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        self.apriltag_topic = rospy.get_param('~apriltag_topic', '/tag_detections')
        self.pallet_topic = rospy.get_param('~pallet_topic', '/pallet_detection')

        rospy.loginfo("獲取訂閱主題參數")
        rospy.loginfo("odom_topic: {}, type: {}".format(self.odom_topic, type(self.odom_topic)))
        rospy.loginfo("apriltag_topic: {}, type: {}".format(self.apriltag_topic, type(self.apriltag_topic)))
        rospy.loginfo("pallet_topic: {}, type: {}".format(self.pallet_topic, type(self.pallet_topic)))

    def create_subscribers(self):
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.apriltag_sub = rospy.Subscriber(self.apriltag_topic, PoseArray, self.apriltag_callback, queue_size=1)
        self.pallet_sub = rospy.Subscriber(self.pallet_topic, Pose, self.pallet_callback, queue_size=1)

    def update_gui(self):
        self.log_info()  # 記錄資訊
        self.pallet_pose_label.config(text="蘋果位置: x={:.3f}, y={:.3f}, theta={:.3f}".format(
            self.fruit_2d_pose_x, self.fruit_2d_pose_y, self.fruit_2d_theta))
        self.pallet_z_pose_label.config(text="蘋果位置: z={:.3f}".format(self.fruit_2d_pose_z))
        self.root.after(100, self.update_gui)  # 每 100ms 更新一次 GUI

    def log_info(self):
        rospy.loginfo("里程計: x={:.3f}, y={:.3f}, theta={:.3f}".format(
            self.robot_2d_pose_x, self.robot_2d_pose_y, self.robot_2d_theta))
        rospy.loginfo("AprilTag 位置: x={:.3f}, y={:.3f}, theta={:.3f}".format(
            self.marker_2d_pose_x, self.marker_2d_pose_y, (self.marker_2d_theta * 180 / math.pi)))
        rospy.loginfo("棧板位置: x={:.3f}, y={:.3f}, theta={:.3f}".format(
            self.fruit_2d_pose_x, self.fruit_2d_pose_y, (self.fruit_2d_theta * 180 / math.pi)))

    def odom_callback(self, msg):
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        if theta < 0:
            theta += math.pi * 2
        if theta > math.pi * 2:
            theta -= math.pi * 2

        self.robot_2d_pose_x = msg.pose.pose.position.x
        self.robot_2d_pose_y = msg.pose.pose.position.y
        self.robot_2d_theta = theta

        d_theta = self.robot_2d_theta - self.previous_robot_2d_theta
        if d_theta > 5.0:
            d_theta -= 2 * math.pi
        elif d_theta < -5.0:
            d_theta += 2 * math.pi

        self.total_robot_2d_theta += d_theta
        self.previous_robot_2d_theta = self.robot_2d_theta
        self.robot_2d_theta = self.total_robot_2d_theta

    def apriltag_callback(self, msg):
        try:
            marker_msg = msg.poses[0]
            quaternion = (marker_msg.orientation.x, marker_msg.orientation.y,
                          marker_msg.orientation.z, marker_msg.orientation.w)
            theta = tf.transformations.euler_from_quaternion(quaternion)[1]
            self.marker_2d_pose_x = -marker_msg.position.z
            self.marker_2d_pose_y = marker_msg.position.x
            self.marker_2d_theta = -theta
        except IndexError:
            pass

    def pallet_callback(self, msg):
        try:
            marker_msg = msg
            quaternion = (marker_msg.orientation.x, marker_msg.orientation.y,
                          marker_msg.orientation.z, marker_msg.orientation.w)
            theta = tf.transformations.euler_from_quaternion(quaternion)[1]
            self.fruit_2d_pose_x = -marker_msg.position.z
            self.fruit_2d_pose_y = marker_msg.position.x
            self.fruit_2d_pose_z = marker_msg.position.y
            self.fruit_2d_theta = -theta
        except:
            pass


def main():
    try:
        PoseVisualization()
        rospy.spin()  # 注意：由於 Tkinter mainloop，這行不會被執行
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()