#include <ros/ros.h>
#include <serial/serial.h>
#include <custom_msgs/CmdCutPliers.h>
#include <sensor_msgs/Image.h>
#include "geometry_msgs/Twist.h"  
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <algorithm> // C++11提供std::min/std::max

using namespace std::chrono_literals;
using std::string;

// 全局串口物件
serial::Serial ros_ser;
uint8_t FLAG_USART; //串口发送标志
uint8_t Flag_OK = 0;


// 全局變數（只控制手臂1）
int32_t S_H1;      // 手臂1目標高度
int32_t S_L1;      // 手臂1目標前伸長度
uint8_t S_En1;     // 手臂1使能狀態（電機1）
uint8_t S_En2;     // 手臂1使能狀態（電機2）
uint8_t S_C1;      // 手臂1爪子狀態

// 用於接收下位機傳回的狀態
int32_t R_H1;      // 實際手臂1高度（單位：毫米）
int32_t R_L1;      // 實際手臂1長度（單位：毫米）
uint8_t R_En1;     // 實際使能狀態（電機1）
uint8_t R_En2;     // 實際使能狀態（電機2）
uint8_t R_C1;      // 實際爪子狀態
int32_t Voltage;   // 電池電壓（單位：整數表示，換算成浮點數）

// 其他全局輔助變數
uint16_t count_1 = 0;
uint16_t a = 0, b = 0;
char aa;  // 用於儲存鍵盤輸入

// forward declaration
void send_data(void);
void receive_and_process_data(void);
void initialize_arms(void);

void chatterCallback(const geometry_msgs::Twist &msg)//获取键盘控制的回调函数
{

    ROS_INFO("X_linear: [%g]", msg.linear.x);//
    ROS_INFO("Y_linear: [%g]", msg.linear.y);//
    ROS_INFO("Z_linear: [%g]", msg.linear.z);//
    ROS_INFO("X_angular: [%g]", msg.angular.x);//
    ROS_INFO("Y_angular: [%g]", msg.angular.y);//
    ROS_INFO("Z_angular: [%g]", msg.angular.z);//
    ROS_INFO("-------------");
	
            if(msg.linear.x>0 && msg.angular.z>0){//按下 U 键
							FLAG_USART=1;
			        }//开启发送指令
       else if(msg.linear.x>0 && msg.angular.z<0){//按下 O 键
              FLAG_USART=0;
			        }//停止发送指令
			 	
}

// -----------------------------------------------------------------------------
// send_data()：打包 S_H1、S_L1、S_En1、S_En2、S_C1 數據並發送到下位機
// 數據帧格式：
//   帧頭：2 bytes (0xAA, 0xAA)
//   功能字：1 byte (0xF1)
//   數據長度：1 byte (固定值 22)
//   S_H1：4 bytes
//   S_L1：4 bytes
//   (後續手臂2相關數據填0)
//   S_En1：1 byte, S_En2：1 byte
//   (使能3,使能4填0)
//   S_C1：1 byte
//   (爪子2填0)
//   校驗和：1 byte（前26個字節的總和）
// 總長度 27 bytes
// -----------------------------------------------------------------------------
void send_data(void)
{
    uint8_t tbuf[27];
    tbuf[26] = 0;  // 校驗和初始化

    tbuf[0] = 0xAA;
    tbuf[1] = 0xAA;
    tbuf[2] = 0xF1;
    tbuf[3] = 22; // 數據長度

    // 封裝 S_H1 (4 bytes)
    tbuf[4]  = S_H1 & 0xFF;
    tbuf[5]  = (S_H1 >> 8) & 0xFF;
    tbuf[6]  = (S_H1 >> 16) & 0xFF;
    tbuf[7]  = (S_H1 >> 24) & 0xFF;

    // 封裝 S_L1 (4 bytes)
    tbuf[8]  = S_L1 & 0xFF;
    tbuf[9]  = (S_L1 >> 8) & 0xFF;
    tbuf[10] = (S_L1 >> 16) & 0xFF;
    tbuf[11] = (S_L1 >> 24) & 0xFF;

    // 手臂2相關數據置0 (8 bytes)
    for (int i = 12; i < 20; i++) {
        tbuf[i] = 0;
    }

    // 封裝使能狀態 S_En1, S_En2 (2 bytes)
    tbuf[20] = S_En1;
    tbuf[21] = S_En2;

    // 使能3,使能4置0 (2 bytes)
    tbuf[22] = 0;
    tbuf[23] = 0;

    // 封裝爪子狀態 S_C1 (1 byte)
    tbuf[24] = S_C1;

    // 爪子2置0 (1 byte)
    tbuf[25] = 0;

    // 計算校驗和
    for (int i = 0; i < 26; i++) {
        tbuf[26] += tbuf[i];
    }

    try {
        ros_ser.write(tbuf, 27);
    } catch (serial::IOException &e) {
        ROS_ERROR("Unable to send data through serial port");
    }
}

/*###################################################################################
  receive_and_process_data() 函數：接收下位機傳回的手臂1狀態數據，並解析校驗
    修改後的數據帧結構總長 16 字節
###################################################################################*/
//************************接收并处理来自下位机的数据**************************// 
//************************接收并处理来自下位机的数据**************************// 
//************************接收并处理来自下位机的数据**************************// 

void receive_and_process_data(void)
{        
//连续获取下位机的数据
			/*<01>*///buffer[4] ;//H1
			/*<02>*///buffer[5] ; 
			/*<03>*///buffer[6] ;
			/*<04>*///buffer[7] ; 
								
			/*<05>*///buffer[8] ;//L1
			/*<06>*///buffer[9] ;
			/*<07>*///buffer[10] ;    
			/*<08>*///buffer[11] ;  
								
			/*<09>*///buffer[12] ;//H2 
			/*<10>*///buffer[13] ; 
			/*<11>*///buffer[14];
			/*<12>*///buffer[15];
								
			/*<13>*///buffer[16];//L2 
			/*<14>*///buffer[17];
			/*<15>*///buffer[18];
      /*<16>*///buffer[19];	
								
      /*<17>*///buffer[20];//EN1
      /*<18>*///buffer[21];//EN2					 
			/*<19>*///buffer[22];//EN3	
			/*<20>*///buffer[23];//EN4
			
			/*<21>*///buffer[24];//C1
			/*<22>*///buffer[25];//C2

			/*<21>*///buffer[26];//Voltage
			/*<22>*///buffer[27];// 
			/*<21>*///buffer[28];// 
			/*<22>*///buffer[29];//
						
       //连续获取下位机的数据				
       size_t n = ros_ser.available();//获取缓冲区内的字节数
       a++;
         if(n>0)  
               {		   
                 uint8_t buffer[30];uint8_t buf[30];
                 
                 if(n>=62){
                   while(n){n = ros_ser.available();if(n>=62)ros_ser.read(buf, 30);else {break;}}//砍掉旧缓存，获取最新数据  ,预防堵塞                 
                 }                 
                 if(n>=31 && n<62){
                     for(uint8_t i=0;i<n;i++){
                         if(buffer[0]!=0XAA)ros_ser.read(buffer, 1);
                         else {break;} 
                     }//逐个读字节，读到帧头跳出
                  }                    
                 if(buffer[0]==0XAA)//
                  {
                   ros_ser.read(buffer, 30);//                 
                   if(buffer[0]==0XAA && buffer[1]==0XF1)
                      {              
                       uint8_t sum=0; 
	               for(uint8_t j=0;j<29;j++)sum+=buffer[j];    //计算校验和	
                       if(buffer[29] == (uint8_t)(sum+buffer[0]))
                          {b++;	Flag_OK=1;
 						                   R_H1  = (int32_t)((buffer[3]<<0) |(buffer[4]<<8) |(buffer[5]<<16) |(buffer[6]<<24));//单位毫米
														   R_L1  = (int32_t)((buffer[7]<<0) |(buffer[8]<<8) |(buffer[9]<<16) |(buffer[10]<<24));//单位毫米

										           R_En1 = buffer[19];
										           R_En2 = buffer[20]; 

 										
						                   R_C1    = buffer[23]; 
														
                               Voltage =(int32_t)((buffer[25]<<0) |(buffer[26]<<8) |(buffer[27]<<16) |(buffer[28]<<24));  	                       										 				              				
	                   } 			  						
                       }
                       buffer[0]=0Xff;buffer[1]=0Xff; 
                   }
		     
                } 
        if(++count_1>4){//显示频率降低
           count_1=0;
                       
           std::cout<< "[01] Current_Height_1:" << (int)R_H1 <<"[mm]"<<std::endl;
           std::cout<< "[02] Current_length_1:" << (int)R_L1 <<"[mm]"<<std::endl;

           std::cout<< "[05] (Height_1)En1:" <<  (int)R_En1  <<std::endl;
           std::cout<< "[06] (length_1)En2:" <<  (int)R_En2  <<std::endl;

           std::cout<< "[09] State_Claw1:" <<   (int)R_C1  <<std::endl;
					
           std::cout<< "[11] Voltage:" << (float)Voltage/100 <<std::endl;//电池电压
                      								 
           std::cout<< "[12] 主循环频数a:" << (uint16_t)a <<std::endl;
           std::cout<< "[13] 有效接收数b:" << (uint16_t)b <<std::endl;                                        
           std::cout<< "[14] a/b:" <<  (float)a/b <<std::endl;
           if(b>5000)b=b/10,a=a/10;
         
           std::cout<< "-----------------------" <<std::endl;           														
                     }                         
}
// -----------------------------------------------------------------------------
// initialize_arms()：初始化手臂1參數，並發送初始數據到下位機
// -----------------------------------------------------------------------------
void initialize_arms()
{
    S_H1 = 0;   // 手臂1高度初始化（最低位置）
    S_L1 = 0;   // 手臂1長度初始化
    S_En1 = 1;  // 啟用電機1
    S_En2 = 1;  // 啟用電機2
    S_C1 = 0;   // 爪子1張開

    send_data();
}

// -----------------------------------------------------------------------------
// MinimalSubscriber 節點：訂閱 "Keyboard" topic 接收鍵盤指令（此處僅示範）
// -----------------------------------------------------------------------------
class MinimalSubscriber
{
public:
    MinimalSubscriber(ros::NodeHandle& nh)
    {
        sub_ = nh.subscribe<sensor_msgs::Image>("Keyboard", 2, &MinimalSubscriber::callback, this);
    }
private:
    ros::Subscriber sub_;
    void callback(const sensor_msgs::Image::ConstPtr& msg)
    {
        // 這裡利用 image 的 height 欄位暫存鍵盤輸入
        aa = msg->height;
    }
};

// -----------------------------------------------------------------------------
// ArmStatusPublisher 節點：發布手臂當前狀態到 /arm_current_status 主題
// -----------------------------------------------------------------------------
class ArmStatusPublisher
{
public:
    ArmStatusPublisher(ros::NodeHandle& nh)
    {
        pub_ = nh.advertise<custom_msgs::CmdCutPliers>("/arm_current_status", 1);
        timer_ = nh.createTimer(ros::Duration(0.1), &ArmStatusPublisher::timerCallback, this);
    }
private:
    ros::Publisher pub_;
    ros::Timer timer_;
    void timerCallback(const ros::TimerEvent&)
    {
        custom_msgs::CmdCutPliers msg;
        msg.height1 = R_H1;
        msg.length1 = R_L1;
        msg.claw1 = (R_C1 != 0);
        pub_.publish(msg);
    }
};

// -----------------------------------------------------------------------------
// CmdCutPliersPublisher 節點：訂閱 /cmd_cut_pliers topic 並更新控制參數
// 此節點會由 timer_callback 定時呼叫 send_data() 將更新的參數發送至下位機。
// -----------------------------------------------------------------------------
class CmdCutPliersPublisher
{
public:
    CmdCutPliersPublisher(ros::NodeHandle& nh)
    {
        // 透過 launch 設定 topic 名稱
        std::string cmd_topic;
        nh.param<std::string>("cmd_topic", cmd_topic, "/cmd_cut_pliers_123"); // 預設 "/cmd_cut_pliers"

        // 初始化手臂1控制參數（僅控制手臂1）
        target_height1_ = 140;
        target_length1_ = -1;
        last_valid_length_ = 10;
        claw1_ = false;
        allow_retract_ = false;

        // 使用變數 cmd_topic，而不是固定的 "/cmd_cut_pliers"
        pub_ = nh.advertise<custom_msgs::CmdCutPliers>(cmd_topic, 10);
        sub_ = nh.subscribe(cmd_topic, 10, &CmdCutPliersPublisher::cmdCutPliersCallback, this);

        timer_ = nh.createTimer(ros::Duration(0.1), &CmdCutPliersPublisher::timerCallback, this);

        // 修正錯誤: 確保 cmd_topic_ 沒有拼錯
        ROS_INFO("Subscribed to topic: %s", cmd_topic.c_str());
    }

private:
    int target_height1_;
    int target_length1_;
    int last_valid_length_;
    bool claw1_;
    bool allow_retract_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Timer timer_;

    void cmdCutPliersCallback(const custom_msgs::CmdCutPliers::ConstPtr& msg)
    {
        bool updated = false;
        // 前進模式：若 mode==0，僅允許 length 增加
        if (msg->mode == 0)
        {
            if (!allow_retract_ && msg->length1 >= last_valid_length_)
            {
                if (target_length1_ < msg->length1)
                {
                    target_length1_ = msg->length1;
                    last_valid_length_ = target_length1_;
                    updated = true;
                }
                else
                {
                    ROS_WARN("Ignoring forward command: target_length (%d) is less than current (%d)", msg->length1, last_valid_length_);
                }
            }
        }
        // 後退模式：若 mode==1，僅允許 length 減少
        else if (msg->mode == 1)
        {
            if (msg->length1 < last_valid_length_)
            {
                target_length1_ = msg->length1;
                last_valid_length_ = msg->length1;
                updated = true;
            }
            else
            {
                ROS_WARN("Ignoring reverse command: target_length (%d) is greater than current (%d)", msg->length1, last_valid_length_);
            }
        }

        // 更新高度（僅控制手臂1）
        if (msg->height1 >= 0 && target_height1_ != msg->height1)
        {
            target_height1_ = msg->height1;
            updated = true;
        }

        // 更新爪子狀態
        if (claw1_ != msg->claw1)
        {
            claw1_ = msg->claw1;
            updated = true;
        }

        if (updated)
        {
            ROS_INFO("Updated target: height=%d, length=%d, mode=%d", target_height1_, target_length1_, msg->mode);
        }
    }

    void timerCallback(const ros::TimerEvent&)
    {
        // 設定全域變數 S_H1 為目標高度（限制在 [0,280]，ROS1版控制手臂1高度以正數表示，0最低，280最高）
        S_H1 = std::min(std::max(target_height1_, 0), 280);
        // 長度：保持最後有效值
        S_L1 = std::min(std::max(last_valid_length_, 10), 440);
        S_C1 = claw1_;
        ROS_INFO("Publishing data: height=%d, length=%d, claw=%s", S_H1, S_L1, S_C1 ? "True" : "False");
        send_data();
    }
};


// -----------------------------------------------------------------------------
// 主函數
// -----------------------------------------------------------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "cut_pliers_controller");
    ros::NodeHandle nh;

    // 初始化串口
    ros_ser.setPort("/dev/ttyUSB0");
    ros_ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    ros_ser.setTimeout(to);
    try {
        ros_ser.open();
    }
    catch (serial::IOException& e) {
        ROS_ERROR("Unable to open serial port");
        return -1;
    }
    if (ros_ser.isOpen())
        ROS_INFO("/dev/ttyUSB0 is opened.");
    else
        return -1;

    // 初始化手臂參數（僅控制手臂1）
    initialize_arms();

    MinimalSubscriber minimal_subscriber(nh);
    CmdCutPliersPublisher cmd_cut_pliers_publisher(nh);
    ArmStatusPublisher arm_status_publisher(nh);

    // 使用 AsyncSpinner 多線程執行所有節點
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Rate loop_rate(150);
    while (ros::ok()) {
        receive_and_process_data();
        loop_rate.sleep();
    }

    // 結束前停止手臂動作
    S_En1 = 0;
    S_En2 = 0;
    S_C1 = 0;
    send_data();
    ros_ser.close();

    ros::shutdown();
    return 0;
}
