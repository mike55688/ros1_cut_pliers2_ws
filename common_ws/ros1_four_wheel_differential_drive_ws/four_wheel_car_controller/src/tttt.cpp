#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

// 用於追蹤是否有新訊息的全域變數
bool new_message_received = false;

// 訂閱者的回呼函數
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    ROS_INFO("Received /cmd_vel message: linear.x = %f, angular.z = %f",
             msg->linear.x, msg->angular.z);
    new_message_received = true;  // 設定為 true 表示有新訊息
}

int main(int argc, char** argv) {
    // 初始化 ROS 節點
    ros::init(argc, argv, "cmd_vel_listener");
    ros::NodeHandle nh;

    // 訂閱 /cmd_vel topic
    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 10, cmdVelCallback);

    ros::Rate rate(10);  // 設定迴圈頻率為 10 Hz

    while (ros::ok()) {
        // 處理 ROS 回呼
        ros::spinOnce();

        if (new_message_received) {
            ROS_INFO("New /cmd_vel message received!");
            new_message_received = false;  // 重置狀態以便檢測下一條訊息
        } else {
            ROS_WARN("No new /cmd_vel messages received.");
        }

        rate.sleep();
    }

    return 0;
}
