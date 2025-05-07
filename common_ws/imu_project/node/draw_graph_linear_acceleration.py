import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import low_pass_filter as lpf

x_angular_velocity_list = []
y_angular_velocity_list = []
z_angular_velocity_list = []

x_linear_acceleration_list = []
y_linear_acceleration_list = []
z_linear_acceleration_list = []

index = count()

x_index = []
y = []


def Imucallback(msg):
    
    x_angular_velocity_list.append(msg.angular_velocity.x)
    y_angular_velocity_list.append(msg.angular_velocity.y)
    z_angular_velocity_list.append(msg.angular_velocity.z)
    x_index.append(next(index))
    
    x_linear_acceleration_list.append(msg.linear_acceleration.x)
    y_linear_acceleration_list.append(msg.linear_acceleration.y)
    z_linear_acceleration_list.append(msg.linear_acceleration.z)
    
    # print(x_angular_velocity_list)
    # print(y_angular_velocity_list)
    # print(z_angular_velocity_list)

def animate(i):
    x_index.append(next(index))
    
    

    plt.subplot(3, 1, 1)
    plt.cla()
    plt.title("IMU Data : X Linear Acceleration")
    plt.xlabel("Time")
    plt.ylabel("Acceleration")
    # plt.xlim(0,xview)
    # plt.ylim(-2,2)
    plt.plot(x_index[:len(x_linear_acceleration_list)], x_linear_acceleration_list)
    
    plt.subplot(3, 1, 2)
    plt.cla()
    plt.title("IMU Data : Y Linear Acceleration")
    plt.xlabel("Time")
    plt.ylabel("Acceleration")
    # plt.ylim(-12,-6)
    plt.plot(x_index[:len(y_linear_acceleration_list)], y_linear_acceleration_list)

    plt.subplot(3, 1, 3)
    plt.cla()
    plt.title("IMU Data : Z Linear Acceleration")
    plt.xlabel("Time")
    plt.ylabel("Acceleration")
    # plt.ylim(-2,2)
    plt.plot(x_index[:len(z_linear_acceleration_list)], z_linear_acceleration_list)
    
    # xview = xview + 200


if __name__ == "__main__":
    rospy.init_node('Imu_read', anonymous=True)
    imu_sub = rospy.Subscriber('/camera/imu', Imu, Imucallback)
    ani = FuncAnimation(plt.gcf(), animate, interval = 100)
    # plt.tight_layout()
    # fig = plt.figure()
    # fig.set_figheight(5)
    # fig.set_figwidth(10)
    plt.grid(True)
    plt.show()
    rospy.spin()
