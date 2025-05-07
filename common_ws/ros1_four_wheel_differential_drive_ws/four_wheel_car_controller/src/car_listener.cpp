/*
*****************************************
*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

void odomCallback(const nav_msgs::Odometry &odom){

    ROS_INFO("X_linear_speed: [%.4f m/s]", odom.twist.twist.linear.x);//X轴线速度 
    ROS_INFO("Y_linear_speed: [%.4f m/s]", odom.twist.twist.linear.y);//Y轴线速度 
    ROS_INFO("Z_angular_speed: [%.4f rad/s]", odom.twist.twist.angular.z);//绕Z轴角速度 

    ROS_INFO("position.x: [%.4f m]", odom.pose.pose.position.x);//X轴行程 
    ROS_INFO("position.y: [%.4f m]", odom.pose.pose.position.y);//y轴行程 
    ROS_INFO("position.z: [%.4f rad]", odom.pose.pose.position.z);//车体z轴转角

    ROS_INFO("-------------");
}

int main(int argc, char **argv){
    ros::init(argc, argv, "car_listener");
    ros::NodeHandle np;//为这个进程的节点创建一个句柄

    ros::Subscriber Odom_sub = np.subscribe("odom", 100, odomCallback);//订阅

    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok())
    {
 

        ROS_INFO("-------------"); 
        ros::spinOnce();//若有则执行回调处理函数，完后继续往下执行
        loop_rate.sleep();//循环延时时间
 

    }
    return 0;

}

