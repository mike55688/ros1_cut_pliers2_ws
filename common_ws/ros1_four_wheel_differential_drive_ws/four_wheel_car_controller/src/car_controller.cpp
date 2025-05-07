/*
*******************************************
*/
#include "ros/ros.h"
#include <serial/serial.h>
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <string>
using namespace std;
//创建一个serial类
serial::Serial sp;

#define  to_rad  0.01745329f  //角度转弧度


uint8_t FLAG_USART; //串口发送标志
uint16_t count_1,count_2, count_3;//计数器

uint8_t Flag_start=0;//下位机运行标志
float x_mid_speed; //
float y_mid_speed; //
float z_mid_speed; //
float z_mid_angle; //
float angle_A,angle_B,angle_C,angle_D;//发送到下位机的4个轮子的角度
float speed_A,speed_B,speed_C,speed_D;//发送到下位机的4个轮子的速度
int size;
float Data_US[12];//发送到下位机的数据数组
float Data_UR[22];//接收来自下位机的数据数组
uint16_t a,b;
void send_data(void);//串口发送协议函数
uint8_t Flag_OK=0;

double Roll = 0.0;    double Pitch = 0.0;    double Yaw = 0.0; double Yaw_zero = 0.0;double Yaw_error = 0.0; double Yaw_last = 0.0;

bool new_message_received = false;
int rate;
string topic_odom, topic_cmd_vel;

typedef unsigned char byte;
float b2f(byte m0, byte m1, byte m2, byte m3)//float 型解算为4个字节
{if ((m0 == 0x00 || m0 == 0x80) && m1 == 0x00 && m2 == 0x00 && m3 == 0x00) return 0;
//求符号位
    float sig = 1.;
    if (m0 >=128.)
        sig = -1.;
  
//求阶码
    float jie = 0.;
     if (m0 >=128.)
    {
        jie = m0-128.  ;
    }
    else
    {
        jie = m0;
    }
    jie = jie * 2.;
    if (m1 >=128.)
        jie += 1.;
  
    jie -= 127.;
//求尾码
    float tail = 0.;
    if (m1 >=128.)
        m1 -= 128.;
    tail =  m3 + (m2 + m1 * 256.) * 256.;
    tail  = (tail)/8388608;   //   8388608 = 2^23

    float f = sig * pow(2., jie) * (1+tail);
 
    return f;
}

void chatterCallback(const geometry_msgs::Twist::ConstPtr &msg)//获取键盘控制的回调函数
{
/*四轮四驱*/  
//	  ROS_INFO("X_linear: [%g]", msg.linear.x);//
//    ROS_INFO("Y_linear: [%g]", msg.linear.y);//
//    ROS_INFO("Z_linear: [%g]", msg.linear.z);//
//    ROS_INFO("X_angular: [%g]", msg.angular.x);//
//    ROS_INFO("Y_angular: [%g]", msg.angular.y);//
//    ROS_INFO("Z_angular: [%g]", msg.angular.z);//
//    ROS_INFO("-------------");
    // last_cmdvelcb_time = ros::Time::now();
    new_message_received = true;

    x_mid_speed =msg->linear.x;//将这个值作为X方向的速度目标
    z_mid_speed =msg->linear.z;//将这个值作为Z旋转时的速度目标
    z_mid_angle =msg->angular.z;//将这个值作为Y方向的速度目标

//	  if(x_mid_speed > +0)x_mid_speed = +0.5;//设置为固定速度 0.5m/s
//    if(x_mid_speed < -0)x_mid_speed = -0.5;

//    if(z_mid_speed > +0)z_mid_speed = +0.5;
//    if(z_mid_speed < -0)z_mid_speed = -0.5;

//    if(z_mid_angle > +0)z_mid_angle = +0.5;
//    if(z_mid_angle < -0)z_mid_angle = -0.5;

	  if(x_mid_speed > +1.1)x_mid_speed = +1.1;//限制最大速度 1.1m/s
    if(x_mid_speed < -1.1)x_mid_speed = -1.1;

    if(z_mid_speed > +1.1)z_mid_speed = +1.1;
    if(z_mid_speed < -1.1)z_mid_speed = -1.1;

    if(z_mid_angle > +1.1)z_mid_angle = +1.1;
    if(z_mid_angle < -1.1)z_mid_angle = -1.1;


            if(x_mid_speed>0 && z_mid_speed==0 && z_mid_angle==0){//按下 I 键 
			               speed_A= x_mid_speed;
				       speed_B= x_mid_speed; 
				       speed_C= x_mid_speed; 
				       speed_D= x_mid_speed; 
                        // printf("i");
			        }//前进
       else if(x_mid_speed<0 && z_mid_speed==0 && z_mid_angle==0){//按下 < 键 
			               speed_A= x_mid_speed;
				       speed_B= x_mid_speed; 
				       speed_C= x_mid_speed; 
				       speed_D= x_mid_speed;
                      //  printf("<");  

			        }//后退

       else if(x_mid_speed==0 && z_mid_speed==0 && z_mid_angle>0){//按下 J 键
			               speed_A= z_mid_angle*0.5F;
				       speed_B= -z_mid_angle*0.5F; 
				       speed_C= -z_mid_angle*0.5F; 
				       speed_D= z_mid_angle*0.5F;
                      //  printf("j");  

			        }//左自传
       else if(x_mid_speed==0 && z_mid_speed==0 && z_mid_angle<0){//按下 L 键
			               speed_A= z_mid_angle*0.5F;
				       speed_B= -z_mid_angle*0.5F; 
				       speed_C= -z_mid_angle*0.5F; 
				       speed_D= z_mid_angle*0.5F;
                      //  printf("l");  

			        }//右自传

       else if(x_mid_speed>0 && z_mid_speed==0 && z_mid_angle>0){//按下 U 键
			               speed_A= x_mid_speed*1.1F;
				       speed_B= x_mid_speed*0.9F; //0.7F 
				       speed_C= x_mid_speed*0.7F; //0.5F
				       speed_D= x_mid_speed*1.0F;  //0.95F
                      //  printf("u");  

			        }//左斜上
       else if(x_mid_speed>0 && z_mid_speed==0 && z_mid_angle<0){//按下 O 键
			               speed_B= x_mid_speed*1.1F;
				       speed_A= x_mid_speed*0.9F; 
				       speed_D= x_mid_speed*0.7F;
				       speed_C= x_mid_speed*1.0F; 
                      //  printf("o");				 

			        }//右斜上
	     else if(x_mid_speed<0 && z_mid_speed==0 && z_mid_angle<0){//按下 M 键
			               speed_A= x_mid_speed;
				       speed_B= x_mid_speed*0.7F; 
				       speed_C= x_mid_speed*0.5F; 
				       speed_D= x_mid_speed*0.95F; 
                      //  printf("m"); 

			        }//左斜下
        else if(x_mid_speed<0 && z_mid_speed==0 && z_mid_angle>0){//按下 > 键
			               speed_B= x_mid_speed;
				       speed_A= x_mid_speed*0.7F; 
				       speed_D= x_mid_speed*0.5F;
				       speed_C= x_mid_speed*0.95F; 
                      //  printf(">"); 

			        }//右斜下
  
   if(x_mid_speed==0 && z_mid_speed==0 && z_mid_angle==0){
        speed_B = 0;
        speed_A = 0;
        speed_D = 0;
        speed_C = 0;
   }
   else {Flag_start=1;FLAG_USART=5;}
							
}


int main(int argc, char **argv){

    ros::init(argc, argv, "listener");

    ros::NodeHandle np, private_np("~");//为这个进程的节点创建一个句柄

    private_np.param<int>("rate", rate, 200);
    private_np.param<string>("topic_cmd_vel", topic_cmd_vel, "cmd_vel");
    private_np.param<string>("topic_odom", topic_odom, "odom");

    ros::Subscriber sub = np.subscribe(topic_cmd_vel, 1000, chatterCallback);//订阅键盘控制

    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>(topic_odom, 50);
    ros::Publisher power_voltage_pub = n.advertise<std_msgs::Float64>("car_voltage", 10);
    std_msgs::Float64 voltage;

    tf::TransformBroadcaster odom_broadcaster;

    double x = 0.0;

    double y = 0.0;

    double th = 0.0;

 

    double vx = 0.0;

    double vy = 0.0;

    double vth = 0.0;

 

    ros::Time current_time, last_time;

    current_time = ros::Time::now();

    last_time = ros::Time::now();

    // last_cmdvelcb_time = ros::Time::now();


    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(115200);
    //串口设置timeout
    sp.setTimeout(to);
 
    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");

    }
    else
    {
        return -1;
    }  
     memset(Data_US, 0, sizeof(uint8_t)*12);		

    for(uint8_t j=0;j<3;j++){
      send_data(); 
      ros::Duration(0.1).sleep(); //
    }		

   ros::Rate loop_rate(rate);//设置循环间隔，即代码执行频率 200 HZ

   while(ros::ok())
   {
     new_message_received = false;
     if(Flag_OK==1){
     float angular_velocity_x = Data_UR[1]*0.001064;//角速度转换成 rad/s
	 float angular_velocity_y = Data_UR[2]*0.001064;//角速度转换成 rad/s
	 float angular_velocity_z = Data_UR[3]*0.001064;//角速度转换成 rad/s			 
	 float accelerated_speed_x = Data_UR[4]/16384;//转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒
	 float accelerated_speed_y = Data_UR[5]/16384;//转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒
	 float accelerated_speed_z = Data_UR[6]/16384;//转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒
		 
		 
	 //设定车子正负方向 ，车子前进方向为X正，后退为X负，左移为Y正，右移为Y负
	 //轮子从上往下看，逆时针转为正角度，顺时针转为负角度
         float Power_A_X =	+Data_UR[8] ;    //A轮X方向速度
         float Power_A_Y =	0 ;    //A轮Y方向速度
		
         float Power_B_X =	-Data_UR[9] ;    //B轮X方向速度
         float Power_B_Y =	0 ;    //A轮Y方向速度
		
         float Power_C_X =	-Data_UR[10] ;    //C轮X方向速度
         float Power_C_Y =	0 ;    //A轮Y方向速度
		
         float Power_D_X =	+Data_UR[11] ;    //D轮X方向速度
         float Power_D_Y =	0 ;    //A轮Y方向速度	

         Roll = Data_UR[13];
         Pitch = Data_UR[14];
         Yaw = Data_UR[7];
    	
    
          vx  = (Power_A_X + Power_B_X + Power_C_X + Power_D_X)/4 ;//底盘当前X方向线速度 m/s	
          vy  = (Power_A_Y + Power_B_Y + Power_C_Y + Power_D_Y)/4 ;//底盘当前Y方向线速度 m/s	
          vth = angular_velocity_z;//设备当前Z轴角速度 rad/s

          current_time = ros::Time::now();//记录当前时间
		
          //以给定机器人速度的典型方式计算里程计
          double dt = (current_time - last_time).toSec();

          double delta_x = (vx * cos(Yaw_error*to_rad) - vy * sin(Yaw_error*to_rad)) * dt;

          double delta_y = (vx * sin(Yaw_error*to_rad) + vy * cos(Yaw_error*to_rad)) * dt;

        //   double delta_th = vth * dt;


        static uint8_t only = 0;
        if(only == 0) Yaw_zero = Yaw, only = 1; //每次运行记录一次初始角度作为零点

        Yaw_error = Yaw - Yaw_zero; //当前角度值减去初始角度值

            x += delta_x;//X轴速度累积位移 m

            y += delta_y;//Y轴速度累积位移 m

            // th += delta_th;//Z轴角速度累积求车体朝向角度  rad //存在漂移
            th = Yaw_error*to_rad;


      //因为所有的里程表都是6自由度的，所以我们需要一个由偏航创建的四元数
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

 
      //首先，我们将通过tf发布转换

      geometry_msgs::TransformStamped odom_trans;

      odom_trans.header.stamp = current_time;

      odom_trans.header.frame_id = topic_odom;

      odom_trans.child_frame_id = "base_link";

 
      odom_trans.transform.translation.x = x;

      odom_trans.transform.translation.y = y;

      odom_trans.transform.translation.z = 0.0;

      odom_trans.transform.rotation = odom_quat;


      //发送转换
      odom_broadcaster.sendTransform(odom_trans);


       //接下来，我们将通过ROS发布里程计信息

       nav_msgs::Odometry odom;

       odom.header.stamp = current_time;

       odom.header.frame_id = topic_odom;

       //设置位置
       odom.pose.pose.position.x = x;

       odom.pose.pose.position.y = y;

       odom.pose.pose.position.z = th;

       odom.pose.pose.orientation = odom_quat;

       //设定速度
       odom.child_frame_id = "base_link";

       odom.twist.twist.linear.x = vx;

       odom.twist.twist.linear.y = vy;

       odom.twist.twist.angular.z = vth;

       //发布消息
       odom_pub.publish(odom);

       last_time = current_time;//保存为上次时间
     }

        ros::spinOnce();//执行回调处理函数，完后继续往下执行
				

            // if (new_message_received) ROS_INFO("New /cmd_vel message received!");
            // else if (!new_message_received) ROS_WARN("No new /cmd_vel messages received.");

		   if(new_message_received){
            count_2=0;
            
				 //若接收到键盘控制，则发送数据到下位机，同时接收下位机发送上来的数据		
		     /*四轮四驱差速*/	
        							  /*<01>*/Data_US[0]  = Flag_start;//电机启动开关，1启动 0停止
							          /*<02>*/Data_US[1]  = speed_A; 
							          /*<03>*/Data_US[2]  = speed_B ; 
							          /*<04>*/Data_US[3]  = speed_C ; 
							          /*<05>*/Data_US[4]  = speed_D ; //ABCD四轮的当前线速度 m/s
							          /*<06>*/Data_US[5]  = 0 ;//预留位 
							          /*<07>*/Data_US[6]  = 0 ;//预留位     
							          /*<08>*/Data_US[7]  = 0 ;//预留位     
							          /*<09>*/Data_US[8]  = 0 ;//预留位 
							          /*<10>*/Data_US[9]  = 0 ;//预留位  
							          /*<11>*/Data_US[10] = 0 ;//预留位 
							          /*<12>*/Data_US[11] = 0 ;//预留位				 
                    //  if(FLAG_USART>0)FLAG_USART--;
				            //  if((current_time - last_cmdvelcb_time).toSec() > 0.3){Data_US[1]=0;Data_US[2]=0;Data_US[3]=0;Data_US[4]=0;}

                     send_data(); //发送指令控制电机运行
		    }
            else if(!new_message_received){  //當沒有收到/cmd_vel新的訊息以後，就停止運動
                count_2++;
                if(count_2 > 50){
                  /*<01>*/Data_US[0]  = Flag_start;//电机启动开关，1启动 0停止
                  /*<02>*/Data_US[1]  = 0; 
                  /*<03>*/Data_US[2]  = 0; 
                  /*<04>*/Data_US[3]  = 0; 
                  /*<05>*/Data_US[4]  = 0; //ABCD四轮的当前线速度 m/s
                  /*<06>*/Data_US[5]  = 0;//预留位 
                  /*<07>*/Data_US[6]  = 0;//预留位     
                  /*<08>*/Data_US[7]  = 0;//预留位     
                  /*<09>*/Data_US[8]  = 0;//预留位 
                  /*<10>*/Data_US[9]  = 0;//预留位  
                  /*<11>*/Data_US[10] = 0;//预留位 
                  /*<12>*/Data_US[11] = 0;//预留位
                  send_data();
                }	
            }

		
          //获取下位机的数据				
               size_t n = sp.available();//获取缓冲区内的字节数
            a++;
         if(n>0)
               {		   
                 uint8_t buffer[64];uint8_t buf[64];
                 
                 if(n>=130){
                   while(n){n = sp.available();if(n>=130)sp.read(buf, 62);else {break;}}//砍掉旧缓存，获取最新数据                   
                 }                 
                 if(n>=65 && n<130){
                     for(uint8_t i=0;i<n;i++){
                         if(buffer[0]!=0XAA)sp.read(buffer, 1);
                         else {break;} 
                     }//逐个读字节，读到帧头跳出
                  }                    
                 if(buffer[0]==0XAA)//
                  {
                   sp.read(buffer, 64);//读出64个字节                   
                   if(buffer[0]==0XAA && buffer[1]==0XF1)
                      {              
                       uint8_t sum=0; 
	               for(uint8_t j=0;j<63;j++)sum+=buffer[j];    //计算校验和	
                       if(buffer[63] == (uint8_t)sum+buffer[0])
                          {b++;	Flag_OK=1;
                           for(uint8_t i=0;i<15;i++)//15个数据
                            {
			     Data_UR[i] = b2f( buffer[4*i+3], buffer[4*i+4], buffer[4*i+5], buffer[4*i+6] );
                            }	                       										 				              				
	                   } 			  						
                       }
                       buffer[0]=0Xff;buffer[1]=0Xff; 
                   }
		     
                }     
                		     
	    /*<00>*///Data_UR[0] ;//电机启动开关，1启动 0停止
			/*<01>*///Data_UR[1] ;//X轴角速度原始数值 
			/*<02>*///Data_UR[2] ;//Y轴角速度原始数值 
			/*<03>*///Data_UR[3] ;//Z轴角速度原始数值
			/*<04>*///Data_UR[4] ;X轴加速度原始数值
			/*<05>*///Data_UR[5] ;Y轴加速度原始数值
			/*<06>*///Data_UR[6] ;Z轴加速度原始数值    
			/*<07>*///Data_UR[7] ;Z轴角度    
			/*<08>*///Data_UR[8] ;A轮速度
			/*<09>*///Data_UR[9] ;B轮速度 
			/*<10>*///Data_UR[10];C轮速度 
			/*<11>*///Data_UR[11];D轮速度
			/*<12>*///Data_UR[12];电压值*100 
			/*<13>*///Data_UR[13]; //预留
			/*<14>*///Data_UR[14]; //预留

		  count_1++;
                  if(count_1>6){//显示频率降低为10HZ
                      count_1=0;
                      if((uint8_t)Data_UR[0]==1){
                          ROS_INFO("[00] Flag_start: [%u ]", (uint8_t)Data_UR[0]);ROS_INFO("[00] Flag_start: ON");}//下位机电机启动/停止标志，1启动，0停止
                      if((uint8_t)Data_UR[0]==0){
                          ROS_INFO("[00] Flag_start: [%u ]", (uint8_t)Data_UR[0]);ROS_INFO("[00] Flag_start: OFF");}//下位机电机启动/停止标志，1启动，0停止

                      ROS_INFO("[01] gyro_Roll: [%d ]",  (int)Data_UR[1]); //X轴角速度原始数据 gyro_Roll
                      ROS_INFO("[02] gyro_Pitch: [%d ]", (int)Data_UR[2]); //Y轴角速度原始数据 gyro_Pitch
                      ROS_INFO("[03] gyro_Yaw: [%d ]",   (int)Data_UR[3]); //Z轴角速度原始数据 gyro_Yaw
								
                      ROS_INFO("[04] accel_x: [%d ]",  (int)Data_UR[4]); //X轴加速度原始数据 accel_x
                      ROS_INFO("[05] accel_y: [%d ]",  (int)Data_UR[5]); //Y轴加速度原始数据 accel_x
                      ROS_INFO("[06] accel_z: [%d ]",  (int)Data_UR[6]); //Z轴加速度原始数据 accel_x 				 

                      ROS_INFO("[07] Yaw: [%.2f deg]",  Data_UR[7]); //绕Z轴角度 deg
													
                      ROS_INFO("[08] Current_linear_A: [%.2f m/s]", +Data_UR[8]); //A轮线速度 m/s
                      ROS_INFO("[09] Current_linear_B: [%.2f m/s]", -Data_UR[9]); //B轮线速度 m/s 
                      ROS_INFO("[10] Current_linear_C: [%.2f m/s]", -Data_UR[10]); //C轮线速度 m/s 
                      ROS_INFO("[11] Current_linear_D: [%.2f m/s]", +Data_UR[11]); //D轮线速度 m/s 	 							
								
                      ROS_INFO("[12] Voltage: [%.2f V]", Data_UR[12]/100); // 电池电压
                      ROS_INFO("[13] Roll: [%.2f deg]", Data_UR[13]); //绕X轴角度 deg
                      ROS_INFO("[14] Pitch: [%.2f deg]", Data_UR[14]); //绕Y轴角度 deg													
													
                      ROS_INFO("a: [%d ]",   a);
                      ROS_INFO("b: [%d ]",   b);                       
											ROS_INFO("a/b: [%.2f ]",   (float)a/(float)b);
                      ROS_INFO("-----------------------"); 
                      if(b>5000)b=b/10,a=a/10;											
                     }					 

        count_3++;
        if(count_3 > 20){
          int number = round(Data_UR[12]);
          voltage.data = (double)number/100;
          power_voltage_pub.publish(voltage);
          count_3=0;
        }
        loop_rate.sleep();//循环延时时间
   }
    

   memset(Data_US, 0, sizeof(uint8_t)*12);
    for(uint8_t j=0;j<3;j++){
      send_data();
      ros::Duration(0.1).sleep(); //
    }

   //关闭串口
   sp.close();  

    return 0;
}


//************************发送12个数据**************************// 

void send_data(void)
{
	  uint8_t len=12;
    uint8_t tbuf[53];

    unsigned char *p;
				
    for(uint8_t i=0;i<len;i++){
	            p=(unsigned char *)&Data_US[i];
        tbuf[4*i+4]=(unsigned char)(*(p+3));
        tbuf[4*i+5]=(unsigned char)(*(p+2));
        tbuf[4*i+6]=(unsigned char)(*(p+1));
        tbuf[4*i+7]=(unsigned char)(*(p+0));
    }						
//fun:功能字 0XA0~0XAF
//data:数据缓存区，48字节
//len:data区有效数据个数

    tbuf[len*4+4]=0;  //校验位置零
    tbuf[0]=0XAA;   //帧头
    tbuf[1]=0XAA;   //帧头
    tbuf[2]=0XF1;    //功能字
    tbuf[3]=len*4;    //数据长度
    for(uint8_t i=0;i<(len*4+4);i++)tbuf[len*4+4]+=tbuf[i]; //计算和校验   
  try
  {
    sp.write(tbuf, len*4+5);//发送数据下位机(数组，字节数)

  }
  catch (serial::IOException& e)   
  {
    ROS_ERROR_STREAM("Unable to send data through serial port"); //如果发送数据失败，打印错误信息
  }
}  

