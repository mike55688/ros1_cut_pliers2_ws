#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import paho.mqtt.client as mqtt
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from gpm_msg.msg import agvmotion
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import math
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np
import json
import datetime
import time

# 設置日期時間的格式
ISOTIMEFORMAT = '%Y-%m-%dT%H:%M:%S.%f+08:00'

class SubProject4:
    def __init__(self,MQTTClient):

        # Set Max list size
        self.MaxListSize = 1000
        
        # Set AGV wheel base and wheel radius
        self.AGVWheelBase = 1.75
        self.AGVWheelRadius = 0.5
        
        # Set IMU initial values
        self.IMUAngularVelocityX = 0.0
        self.IMUAngularVelocityY = 0.0
        self.IMUAngularVelocityZ = 0.0
        self.IMULinearAccelerationX = 0.0
        self.IMULinearAccelerationY = 0.0
        self.IMULinearAccelerationZ = 0.0

        # Set odometry initial values
        self.OdometryPreviousX = 0.0
        self.OdometryPreviousY = 0.0
        self.OdometryPreviousYaw = 0.0

        # Set transform values
        self.IMURollAngle = 0.0
        self.IMUPitchAngle = 0.0
        self.IMUYawAngle = 0.0

        self.LongitudinalVelocity = 0.0
        self.GravityCenter = 0.0
        self.SteeringAngle = 0.0
        
        # Set the upper and lower limit threshold
        self.RollAngleUpperLimitThreshold = 0.01
        self.RollAngleLowerLimitThreshold = -0.01
        
        self.LongitudinalVelocityUpperLimitThreshold = 1.0
        self.LongitudinalVelocityLowerLimitThreshold = -1.0
        
        self.GravityCenterUpperLimitThreshold = 0.001
        self.GravityCenterLowerLimitThreshold = -0.001
        
        self.SteeringAngleUpperLimitThreshold = 90.0
        self.SteeringAngleLowerLimitThreshold = -90.0
        
        # Set time
        
        self.deltatime = 0.0
        self.LastTime = 0.0
        
        self.Odomdeltatime = 0.0
        self.OdomLastTime = 0.0
        
        # Set time, roll angle, longitudinal velocity, gravity center, steering angle list
        
        self.IMUTime = []
        self.OdomTime = []
        self.AGVTime = []
        self.RollAngleData = []
        self.LongitudinalVelocityData = []
        self.GravityCenterData = []
        self.SteeringAngleData = []
        
        # Set Kalman filter parameter
        self.filterX = KalmanFilter(dim_x=4, dim_z=2)
        self.filterX.x = np.array([[0.0], [0.0], [0.0], [0.0]])
        self.filterX.F = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        self.filterX.H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        self.filterX.P = np.diag([1000.0, 1000.0, 1000.0, 1000.0])
        self.filterX.R = np.array([[0.1, 0.0], [0.0, 0.1]])
        self.filterX.Q = Q_discrete_white_noise(dim=4, dt=0.1, var=0.1)

        self.filterY = KalmanFilter(dim_x=4, dim_z=2)
        self.filterY.x = np.array([[0.0], [0.0], [0.0], [0.0]])
        self.filterY.F = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        self.filterY.H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        self.filterY.P = np.diag([1000.0, 1000.0, 1000.0, 1000.0])
        self.filterY.R = np.array([[0.1, 0.0], [0.0, 0.1]])
        self.filterY.Q = Q_discrete_white_noise(dim=4, dt=0.1, var=0.1)

        self.filterZ = KalmanFilter(dim_x=4, dim_z=2)
        self.filterZ.x = np.array([[0.0], [0.0], [0.0], [0.0]])
        self.filterZ.F = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        self.filterZ.H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        self.filterZ.P = np.diag([1000.0, 1000.0, 1000.0, 1000.0])
        self.filterZ.R = np.array([[0.1, 0.0], [0.0, 0.1]])
        self.filterZ.Q = Q_discrete_white_noise(dim=4, dt=0.1, var=0.1)
        
        self.i = 0
        self.temp = 0
         
        # Set IMU, Controller Subscriber
        # self.IMUSubscriber = rospy.Subscriber("/camera/imu", Imu, self.imu_callback)
        self.IMUSubscriber = rospy.Subscriber("/cameraUP/imu", Imu, self.imu_callback)
        self.OdometrySubscriber = rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        # self.OdometrySubscriber = rospy.Subscriber("/rtabmap/odom", Odometry, self.odometry_callback)
        self.AGVMotionSubscriber = rospy.Subscriber("/agvmotion", agvmotion, self.agvmotion_callback)
        
        self.MQTTClient = MQTTClient
        
        self.MQTT_IMUAngularVelocityX = []
        self.MQTT_IMUAngularVelocityY = []
        self.MQTT_IMUAngularVelocityZ = []
        self.MQTT_IMULinearAccelerationX = []
        self.MQTT_IMULinearAccelerationY = []
        self.MQTT_IMULinearAccelerationZ = []
        self.MQTT_IMURollAngle = []
        self.MQTT_IMUPitchAngle = []
        self.MQTT_IMUYawAngle = []
        self.MQTT_Time = []
        
    def imu_callback(self, data):

        self.deltatime = data.header.stamp.to_sec() - self.LastTime if self.deltatime is not None else 0.0
        self.LastTime = data.header.stamp.to_sec() 
        # print(data.header.stamp.to_sec(),"-",self.LastTime,"=",self.deltatime)
        # Filter IMU 6-Axis Data
        self.filterX.predict()
        self.filterX.update([[data.angular_velocity.x],[data.linear_acceleration.x]])
        self.filterY.predict()
        self.filterY.update([[data.angular_velocity.y],[data.linear_acceleration.y]])
        self.filterZ.predict()
        self.filterZ.update([[data.angular_velocity.z],[data.linear_acceleration.z]])
        
        self.IMUAngularVelocityX = self.filterX.x[0][0]
        self.IMUAngularVelocityY = self.filterY.x[0][0]
        self.IMUAngularVelocityZ = self.filterZ.x[0][0]
        self.IMULinearAccelerationX = self.filterX.x[1][0]
        self.IMULinearAccelerationY = self.filterY.x[1][0]
        self.IMULinearAccelerationZ = self.filterZ.x[1][0]
    
        # self.IMURollAngle = self.IMUAngularVelocityZ * self.deltatime * 180 / math.pi
        # self.IMUPitchAngle = self.IMUAngularVelocityX * self.deltatime * 180 / math.pi
        # self.IMUYawAngle = self.IMUAngularVelocityY * self.deltatime * 180 / math.pi
        
        self.IMURollAngle = (math.asin(self.IMULinearAccelerationX / 9.705) * 180 / math.pi) * -1
        
        if(abs(self.IMULinearAccelerationY / 9.705) > 1):
            self.IMUPitchAngle = 0.0
        else:
            self.IMUPitchAngle = math.asin(self.IMULinearAccelerationY / 9.75) * 180 / math.pi + 90
        
        
        if(abs(self.IMURollAngle) > 100):
            self.IMURollAngle = 0.0
        
        # print(self.IMURollAngle,self.IMUPitchAngle,self.IMUYawAngle)
        GravityCentertemp = 0.5 * (9.703 + self.IMULinearAccelerationY) * self.deltatime ** 2
        if(abs(GravityCentertemp) > 100):
            GravityCentertemp = 0.0
        self.GravityCenter += GravityCentertemp

        if(abs(self.LongitudinalVelocity) < 0.1 and abs(self.IMURollAngle) < 0.005):
            self.GravityCenter *= 0.99 
                   
        # Append List
        self.IMUTime.append(data.header.stamp.to_sec())
        self.RollAngleData.append(self.IMURollAngle)
        self.GravityCenterData.append(self.GravityCenter)

        t = datetime.datetime.now().strftime(ISOTIMEFORMAT)
        self.MQTT_IMUAngularVelocityX.append(self.IMUAngularVelocityX)
        self.MQTT_IMUAngularVelocityY.append(self.IMUAngularVelocityY)
        self.MQTT_IMUAngularVelocityZ.append(self.IMUAngularVelocityZ)        
        self.MQTT_IMULinearAccelerationX.append(self.IMULinearAccelerationX)
        self.MQTT_IMULinearAccelerationY.append(self.IMULinearAccelerationY + 9.8)
        self.MQTT_IMULinearAccelerationZ.append(self.IMULinearAccelerationZ)
        self.MQTT_IMURollAngle.append(self.IMURollAngle)
        self.MQTT_IMUPitchAngle.append(self.IMUPitchAngle)
        self.MQTT_IMUYawAngle.append(self.IMUYawAngle)
        self.MQTT_Time.append(t)
        
        if(len(self.MQTT_Time) > 20):
            # MQTT Publish
            
            # Sensor Info
            payload = [  {'SensorName': 'IMU', 'IP': '192.168.0.100', 'Port': 0, 'SensorType': 'IMU', 'DataUnit': 'G', 'IsOnlySaveData': False, 'CustomName': 'name'},
                         {'SensorName': 'IMUAngle', 'IP': '192.168.0.101', 'Port': 0, 'SensorType': 'IMU', 'DataUnit': 'G', 'IsOnlySaveData': False, 'CustomName': 'name'},
                         {'SensorName': 'Odometry', 'IP': '192.168.0.102', 'Port': 0, 'SensorType': '3D-LiDAR', 'DataUnit': 'G', 'IsOnlySaveData': False, 'CustomName': 'name'},
                         {'SensorName': 'ControllerFeedback', 'IP': '192.168.0.103', 'Port': 0, 'SensorType': 'Controller', 'DataUnit': 'G', 'IsOnlySaveData': False, 'CustomName': 'name'}]

            ################################################################
            
            self.MQTTClient.publish("GPM/AGV/SensorList", json.dumps(payload))
            payload = {'SensorName': 'IMU', 'ConnectStatus': True, 'LastUpdateTime': t}
            self.MQTTClient.publish("GPM/AGV/UpdateSensorStatus", json.dumps(payload))
            
            # IMU Raw Data
            Dict_ListRawData = {'Angular Velocity X': self.MQTT_IMUAngularVelocityX, 'Angular Velocity Y': self.MQTT_IMUAngularVelocityY, 'Angular Velocity Z': self.MQTT_IMUAngularVelocityZ,
                                'Linear Acceleration X': self.MQTT_IMULinearAccelerationX, 'Linear Acceleration Y (Gravity Center)': self.MQTT_IMULinearAccelerationY, 'Linear Acceleration Z': self.MQTT_IMULinearAccelerationZ}
            payload = {'SensorName': 'IMU', 'TimeLog': t, 'Dict_ListRawData': Dict_ListRawData, 'IsArrayData': True, 'List_TimeLog': self.MQTT_Time}
            self.MQTTClient.publish("GPM/AGV/IMU", json.dumps(payload))
            
            ################################################################
            
            payload = {'SensorName': 'IMUAngle', 'ConnectStatus': True, 'LastUpdateTime': t}
            self.MQTTClient.publish("GPM/AGV/UpdateSensorStatus", json.dumps(payload))
            
            # IMU Angle Raw Data
            Dict_ListRawData = {'Roll Angle': self.MQTT_IMURollAngle, 'Pitch Angle': self.MQTT_IMUPitchAngle, 'Yaw Angle': self.MQTT_IMUYawAngle}
            payload = {'SensorName': 'IMUAngle', 'TimeLog': t, 'Dict_ListRawData': Dict_ListRawData, 'IsArrayData': True, 'List_TimeLog': self.MQTT_Time}
            self.MQTTClient.publish("GPM/AGV/IMUAngle", json.dumps(payload))
            
            ################################################################
            
            self.MQTT_IMUAngularVelocityX.clear()
            self.MQTT_IMUAngularVelocityY.clear()
            self.MQTT_IMUAngularVelocityZ.clear()
            self.MQTT_IMULinearAccelerationX.clear()
            self.MQTT_IMULinearAccelerationY.clear()
            self.MQTT_IMULinearAccelerationZ.clear()
            self.MQTT_IMURollAngle.clear()
            self.MQTT_IMUPitchAngle.clear()
            self.MQTT_IMUYawAngle.clear()
            self.MQTT_Time.clear()

    def odometry_callback(self, data):
        self.Odomdeltatime = data.header.stamp.to_sec() - self.OdomLastTime if self.Odomdeltatime is not None else 0.0
        self.OdomLastTime = data.header.stamp.to_sec() 
            
        # Calculate longitudinal velocity
        
        displacement = math.sqrt(math.pow(data.pose.pose.position.x - self.OdometryPreviousX, 2) + math.pow(data.pose.pose.position.y - self.OdometryPreviousY, 2))
        self.LongitudinalVelocity = displacement / self.Odomdeltatime

        # Calculate steering angle
        q = data.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        deltayaw = yaw - self.OdometryPreviousYaw if self.OdometryPreviousYaw is not None else 0.0
        # self.SteeringAngle = deltayaw * self.AGVWheelBase / self.AGVWheelRadius

        # Append List
        self.LongitudinalVelocityData.append(self.LongitudinalVelocity)
        # self.SteeringAngleData.append(self.SteeringAngle)

        # Save odometry coordinates
        self.OdometryPreviousX = data.pose.pose.position.x
        self.OdometryPreviousY = data.pose.pose.position.y
        self.OdometryPreviousYaw = yaw

        self.OdomTime.append(data.header.stamp.to_sec())

        # MQTT Publish
        t = datetime.datetime.now().strftime(ISOTIMEFORMAT)
        # Sensor Info
        # payload = [{'SensorName': 'Odometry', 'IP': '192.168.0.100', 'Port': 0, 'SensorType': '3D-LiDAR', 'DataUnit': 'G', 'IsOnlySaveData': False, 'CustomName': 'name'}]
        # print (json.dumps(payload))
        # 要發布的主題和內容
        # self.MQTTClient.publish("GPM/AGV/SensorList", json.dumps(payload))

        # Sensor Status
        payload = {'SensorName': 'Odometry', 'ConnectStatus': True, 'LastUpdateTime': t}
        # print(json.dumps(payload))
        # 要發布的主題和內容
        self.MQTTClient.publish("GPM/AGV/UpdateSensorStatus", json.dumps(payload))

        # Sensor Raw Data
        raw_data_dict = {'Longitudinal Velocity': self.LongitudinalVelocity}
        payload = {'SensorName': 'Odometry', 'TimeLog': t, 'Dict_RawData': raw_data_dict, 'IsArrayData': False}
        self.MQTTClient.publish("GPM/AGV/Odometry", json.dumps(payload))        
        
    def agvmotion_callback(self, data):
        self.SteeringAngle = data.wheelangle
        self.SteeringAngleData.append(self.SteeringAngle)
        self.i += 1 
        self.AGVTime.append(self.i)

        # MQTT Publish
        t = datetime.datetime.now().strftime(ISOTIMEFORMAT)
        # Sensor Info
        # payload = [{'SensorName': 'Controller Feedback', 'IP': '192.168.0.100', 'Port': 0, 'SensorType': 'Controller', 'DataUnit': 'G', 'IsOnlySaveData': False, 'CustomName': 'name'}]
        # print (json.dumps(payload))
        # 要發布的主題和內容
        # self.MQTTClient.publish("GPM/AGV/SensorList", json.dumps(payload))

        # Sensor Status
        payload = {'SensorName': 'ControllerFeedback', 'ConnectStatus': True, 'LastUpdateTime': t}
        # print(json.dumps(payload))
        # 要發布的主題和內容
        self.MQTTClient.publish("GPM/AGV/UpdateSensorStatus", json.dumps(payload))

        # Sensor Raw Data
        raw_data_dict = {'Steering Angle': self.SteeringAngle}
        payload = {'SensorName': 'ControllerFeedback', 'TimeLog': t, 'Dict_RawData': raw_data_dict, 'IsArrayData': False}
        self.MQTTClient.publish("GPM/AGV/ControllerFeedback", json.dumps(payload))


    def update_plot(self):
        # Keep list limited to 1000 elements
        while(len(self.IMUTime) > self.MaxListSize):
            self.IMUTime.pop(0)
            self.RollAngleData.pop(0)
            self.GravityCenterData.pop(0)
            
        while(len(self.OdomTime) > self.MaxListSize):
            self.OdomTime.pop(0)
            self.LongitudinalVelocityData.pop(0)
        
        while(len(self.AGVTime) > self.MaxListSize):
            self.AGVTime.pop(0)
            self.SteeringAngleData.pop(0)
        
        # plt.clf()
        # plt.suptitle('SubProject4')

        # plt.subplot(4,1,1)
        # plt.title("Roll Angle")
        # plt.plot(self.IMUTime, self.RollAngleData, 'mediumspringgreen')
        # plt.axhline(y=self.RollAngleUpperLimitThreshold, color='r', linestyle='--')
        # plt.axhline(y=self.RollAngleLowerLimitThreshold, color='r', linestyle='--')
        # plt.xlabel('Time (s)')
        # plt.ylabel('Angle (Degree)')
        
        # plt.subplot(4,1,2)
        # plt.title("Gravity Center")
        # plt.plot(self.IMUTime, self.GravityCenterData, 'lightsalmon')
        # plt.axhline(y=self.GravityCenterUpperLimitThreshold, color='r', linestyle='--')
        # # plt.axhline(y=self.GravityCenterLowerLimitThreshold, color='r', linestyle='--')
        # plt.xlabel('Time (s)')
        # plt.ylabel('Height (m)')

        # plt.subplot(4,1,3)
        # plt.title("Longitudinal Velocity")
        # plt.plot(self.OdomTime, self.LongitudinalVelocityData, 'deepskyblue')
        # # plt.axhline(y=self.LongitudinalVelocityUpperLimitThreshold, color='r', linestyle='--')
        # # plt.axhline(y=self.LongitudinalVelocityLowerLimitThreshold, color='r', linestyle='--')
        # plt.xlabel('Time (s)')
        # plt.ylabel('Velocity (m/s)')

        # plt.subplot(4,1,4)
        # plt.title("Steering Angle")
        # plt.plot(self.AGVTime, self.SteeringAngleData, 'gray')
        # # plt.axhline(y=self.SteeringAngleUpperLimitThreshold, color='r', linestyle='--')
        # # plt.axhline(y=self.SteeringAngleLowerLimitThreshold, color='r', linestyle='--')
        # plt.xlabel('Time (s)')
        # plt.ylabel('Angle (Degree)')

        # plt.subplots_adjust(hspace=0.6)
        # plt.draw()
        # plt.pause(0.001)        

if __name__ == '__main__':

    # Connect to MQTT broker
    MQTTClient = mqtt.Client()
    MQTTClient.username_pw_set("Rick","dQw4w9WgXcQ")
    MQTTClient.connect("192.168.1.100", 1883, 60)
    MQTTClient.loop_start()

    # Initialize ROS node and Listener object
    rospy.init_node('SubProject4')
    rospy.loginfo("Start Monitoring")
    Listener = SubProject4(MQTTClient)
    Listener.LastTime = rospy.Time.now().to_sec()

    # Create Matplotlib plot
    plt.ion()
    plt.figure(figsize=(10, 8))

    while not rospy.is_shutdown():
        Listener.update_plot()
        plt.pause(0.1)    

    MQTTClient.loop_stop()
    MQTTClient.disconnect()

    if(rospy.is_shutdown()):
        rospy.logwarn("End Monitoring")

    


        
        