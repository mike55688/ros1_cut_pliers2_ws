#include "ros/ros.h"
#include "std_msgs/String.h"
#include <termios.h>
#include <unistd.h>
#include <iostream>

// 用於讀取鍵盤輸入的函數
char getKey() {
    char c;
    struct termios old_attr, new_attr;

    // 獲取當前終端設置
    tcgetattr(STDIN_FILENO, &old_attr);

    // 設置新的終端屬性
    new_attr = old_attr;
    new_attr.c_lflag &= ~(ICANON | ECHO); // 關閉回顯和行緩衝
    tcsetattr(STDIN_FILENO, TCSANOW, &new_attr);

    // 讀取按鍵
    read(STDIN_FILENO, &c, 1);

    // 恢復終端設置
    tcsetattr(STDIN_FILENO, TCSANOW, &old_attr);

    return c;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "keyboard_publisher");
    ros::NodeHandle nh;

    ros::Publisher keyboard_pub = nh.advertise<std_msgs::String>("Keyboard", 10);

    std::cout << "有效按鍵列表:\n"
                 "Q  W  E  T     U  I  O\n"
                 "          G  H  J  K  L\n"
                 " Z  X  C   B     M  ,  .\n"
                 "按下按鍵: ";

    ros::Rate loop_rate(10); // 設置發佈頻率為 10Hz

    while (ros::ok()) {
        char key = getKey(); // 獲取按鍵輸入
        std_msgs::String msg;
        msg.data = std::string(1, key); // 將按鍵轉換為 ROS 字串訊息
        keyboard_pub.publish(msg);     // 發布訊息
        ROS_INFO("Pressed key: %s", msg.data.c_str());
        loop_rate.sleep();
    }

    return 0;
}
