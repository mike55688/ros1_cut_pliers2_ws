#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import tkinter as tk
from tkinter import *
from tkinter.constants import CENTER
import subprocess
import rospy
from nav_msgs.msg import Odometry
from gpm_msg.msg import agvmotion
from gpm_msg.msg import forklift
from gpm_msg.msg import forkposition
import forklift_server.msg
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


class ScriptExecutor():
    def __init__(self):
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size = 1)
        self.sub_agvmotion = rospy.Subscriber("/agvmotion", agvmotion, self.agvmotion_callback, queue_size = 1)
        self.sub_cmd_fork = rospy.Subscriber("/cmd_fork", forklift, self.cmd_fork_callback, queue_size = 1)
        self.init_param()
        self.window()
        
          
    def init_param(self):
        self.test = 0.0
        self.odom_position_x = 0.0
        self.odom_position_y = 0.0
        self.odom_orientation_w = 0.0
        self.odom_orientation_z = 0.0
        self.agv_wheelvelocity = 0.0
        self.agv_wheelangle = 0.0
        self.fork_forwardbackwardposition = 0.0
        self.fork_updownposition = 0.0

    def odom_callback(self,msg):
        self.odom_position_x = msg.pose.pose.position.x
        self.odom_position_y = msg.pose.pose.position.y
        self.odom_orientation_w = msg.pose.pose.orientation.w
        self.odom_orientation_z = msg.pose.pose.orientation.z

    def agvmotion_callback(self,msg):
        self.agv_wheelvelocity = msg.wheelvelocity
        self.agv_wheelangle = msg.wheelangle
        self.fork_forwardbackwardposition = msg.forwardbackpostion
        self.fork_updownposition = msg.updownposition  
        
    def cmd_fork_callback(self, msg):
        self.agv_forkmotion = msg.forkmotion
        #1  停止
        #2  上升
        #3  下降
        #4  前伸
        #5  後縮
        #6  前頃
        #7  後縮
        #other 停止
    
    # def forkpos_callback(self, msg):
    #     self.fork_forwardbackwardposition = msg.forwardbackposition
    #     self.fork_updownposition = msg.updownposition        

    
    def window(self):
        self.window = tk.Tk()
        self.window.title("Script Executor")
        self.window.geometry('1024x800')
        # def update_window_once(self):
        #     self.display_once()
        
        self.display_once()
        
        self.labels = {
            'Odometry Position X': [0, 400],
            'Odometry Position Y': [0, 430],
            'Odometry Orientation W': [0, 460],
            'Odometry Orientation Z': [0, 490],
            'Wheel Velocity':[300, 400],
            'Wheel Angle':[300, 430],
            'Fork Horizontal Position':[600, 400],
            'Fork Vertical Position':[600, 430]
        }
        for key, value in self.labels.items():
            self.labels[key] = [tk.Label(
                self.window, 
                text=f"{key.replace('_', ' ')}: ",
                font=("Times New Romen", 12)), tk.Label(
                    self.window, 
                    text="",
                    font=("Times New Romen", 12))]
            self.labels[key][0].place(x=value[0], y=value[1])
            self.labels[key][1].place(x=value[0]+230, y=value[1])
        while not rospy.is_shutdown():
            self.update_window()
            self.window.update()
            rospy.sleep(0.05) # Set the desired update rate
        self.window.destroy()
        # self.update_window()
        # self.window.mainloop()

    def update_window(self):
        update_values = {
            'Odometry Position X': "%.3f" % self.odom_position_x,
            'Odometry Position Y': "%.3f" % self.odom_position_y,
            'Odometry Orientation W': "%.3f" % self.odom_orientation_w,
            'Odometry Orientation Z': "%.3f" % self.odom_orientation_z,
            'Wheel Velocity': "%.3f" % self.agv_wheelvelocity,
            'Wheel Angle': "%.3f" % self.agv_wheelangle,
            'Fork Horizontal Position': "%.3f" % self.fork_forwardbackwardposition,
            'Fork Vertical Position': "%.3f" % self.fork_updownposition
        }
        for key, value in update_values.items():
            self.labels[key][1].configure(text=value)
    
    def execute_script_1_1(self):
        subprocess.call(['/home/user/shellscript/3DLiDARProject.sh'])

    def execute_script_1_2(self):
        subprocess.call(['/home/user/shellscript/3DLiDARProject_Demo.sh'])

    def execute_script_2(self):
        subprocess.call(['/home/user/shellscript/VisualProject.sh'])

    def execute_script_3(self):
        subprocess.call(['/home/user/shellscript/AntiRolloverControl.sh'])

    
    def update_window_once(self):
        self.display_once()


    def display_once(self):
        self.title = tk.Label(
            text="GPM Autonomous Forklift", 
            font=("Times New Romen", 36, "bold"), 
            padx=5, 
            pady=5, 
            fg="black")
        self.title.pack()
        self.button1_1 = tk.Button(
            self.window, 
            text="3D-LiDAR Navigation + Visual Servoing (Only Servoing)", 
            font=("Times New Romen", 20, "bold"), 
            command=self.execute_script_1_1)
        self.button1_1.pack()

        self.button1_2 = tk.Button(
            self.window, 
            text="3D-LiDAR Navigation + Visual Servoing (Demo Version)", 
            font=("Times New Romen", 20, "bold"), 
            command=self.execute_script_1_2)
        self.button1_2.pack()

        self.button2 = tk.Button(
            self.window, 
            text="3D-Vision Navigation + Visual Servoing (Demo Version)", 
            font=("Times New Romen", 20, "bold"), 
            command=self.execute_script_2)
        self.button2.pack()

        self.button3 = tk.Button(
            self.window, 
            text="Hybrid Fuzzy-Fuzzy Stability Control of Anti-Rollover", 
            font=("Times New Romen", 20, "bold"), 
            command=self.execute_script_3)
        self.button3.pack()
        
        self.title2 = tk.Label(
            text="Status", 
            font=("Times New Romen", 36, "bold"), 
            padx=5, 
            pady=70, 
            fg="black")
        self.title2.pack()        
        

    
if __name__ == '__main__':
    rospy.init_node('ScriptExecutor')
    ScriptExecutor = ScriptExecutor()