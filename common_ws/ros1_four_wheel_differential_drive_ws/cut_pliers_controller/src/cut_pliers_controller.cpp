/*
*******************************************
*/
#include "ros/ros.h"
#include <serial/serial.h>
#include <iostream>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
//创建一个serial类
serial::Serial sp;

#define to_rad  0.017453f  //角度转弧度


uint8_t FLAG_USART; //串口发送标志
uint16_t count_1,count_2;//计数器
uint8_t Flag_OK=0;



int size;
int Voltage;


uint16_t a,b;
void send_data(void);//串口发送协议函数
void initialize_arms();
void receive_and_process_data(void);
int32_t S_H1;
int32_t S_L1;
int32_t S_H2;
int32_t S_L2;
uint8_t S_En1;
uint8_t S_En2;
uint8_t S_En3;
uint8_t S_En4;
uint8_t S_C1;
uint8_t S_C2;

int32_t R_H1;
int32_t R_L1;
int32_t R_H2;
int32_t R_L2;
uint8_t R_En1;
uint8_t R_En2;
uint8_t R_En3;
uint8_t R_En4;
uint8_t R_C1;
uint8_t R_C2;

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


int main(int argc, char **argv){

    ros::init(argc, argv, "listener");

    ros::NodeHandle np;//为这个进程的节点创建一个句柄

    ros::Subscriber sub = np.subscribe("cmd_vel", 200, chatterCallback);//订阅键盘控制

    
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
	
	
   ros::Rate loop_rate(100);//设置循环间隔，即代码执行频率 100 HZ
   initialize_arms();

   while(ros::ok())
   {	

       ros::spinOnce();//执行回调处理函数，完后继续往下执行
	     receive_and_process_data(); //接收并处理来自下位机的数据   
       S_H1=150;//手臂1的高度 
       S_L1=250;//手臂1的长度
       S_H2=150;//手臂2的高度
       S_L2=250;//手臂2的长度
       S_En1=1;//电机1使能(控制手臂1的高度)
       S_En2=1;//电机2使能(控制手臂1的长度)
       S_En3=1;//电机3使能(控制手臂2的高度)
       S_En4=1;//电机4使能(控制手臂2的长度)
       S_C1=1;//爪子1 开/合 0/1
       S_C2=0;//爪子2 开/合 0/1
		 
       if(FLAG_USART==1)send_data(); //发送指令控制电机运行		  
		
       //连续获取下位机的数据				
      //  size_t n = sp.available();//获取缓冲区内的字节数
      //  a++;
      //    if(n>0)
      //          {		   
      //            uint8_t buffer[31];uint8_t buf[31];
                 
      //            if(n>=62){
      //              while(n){n = sp.available();if(n>=62)sp.read(buf, 24);else {break;}}//砍掉旧缓存，获取最新数据                   
      //            }                 
      //            if(n>=31){
      //                for(uint8_t i=0;i<n;i++){
      //                    if(buffer[0]!=0XAA)sp.read(buffer, 1);
      //                    else {break;} 
      //                }//逐个读字节，读到第一个帧头跳出
			// 							n = sp.available(); 
      //             }                    
      //            if(buffer[0]==0XAA&&n>=30)//
      //             {
      //              sp.read(buffer, 30);//读出剩余的30个字节                   
      //              if(buffer[0]==0XAA && buffer[1]==0XF1)
      //                 {              
      //                  uint8_t sum=0;
											
	    //                  for(uint8_t j=0;j<29;j++)sum+=buffer[j];    //计算校验和	

      //                  if(buffer[2] == 26 && buffer[29] == (uint8_t)(sum + 0XAA))
      //                     {b++;	
 			// 			                   R_H1  = (int32_t)((buffer[3]<<0) |(buffer[4]<<8) |(buffer[5]<<16) |(buffer[6]<<24));//单位毫米
			// 											   R_L1  = (int32_t)((buffer[7]<<0) |(buffer[8]<<8) |(buffer[9]<<16) |(buffer[10]<<24));//单位毫米
			// 												 R_H2  = (int32_t)((buffer[11]<<0)|(buffer[12]<<8)|(buffer[13]<<16)|(buffer[14]<<24));//单位毫米
			// 												 R_L2  = (int32_t)((buffer[15]<<0)|(buffer[16]<<8)|(buffer[17]<<16)|(buffer[18]<<24));//单位毫米
										
			// 							           R_En1 = buffer[19];
			// 							           R_En2 = buffer[20]; 
			// 							           R_En3 = buffer[21]; 
			// 							           R_En4 = buffer[22];
 										
			// 			                   R_C1    = buffer[23]; 
			// 											   R_C2    = buffer[24];
														
      //                          Voltage =(int32_t)((buffer[25]<<0) |(buffer[26]<<8) |(buffer[27]<<16) |(buffer[28]<<24));														
	    //                     } 			  						
      //                  }
      //                  buffer[0]=0Xff;buffer[1]=0Xff;
      //              }
		     
      //           } 
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
		  //count_1++;
                  // if(count_1>10){//显示频率降低为10HZ
                  //     count_1=0;

                  //     ROS_INFO("[01] Current_Height_1: [%d  mm]", R_H1);

                  //     ROS_INFO("[02] Current_length_1: [%d  mm]", R_L1);

                  //     ROS_INFO("[03] Current_Height_2: [%d  mm]", R_H2);

                  //     ROS_INFO("[04] Current_length_2: [%d  mm]", R_L2);

                  //     ROS_INFO("[05] En1: [%d  ]", R_En1);

                  //     ROS_INFO("[06] En2: [%d  ]", R_En2);

                  //     ROS_INFO("[07] En3: [%d  ]", R_En3);

                  //     ROS_INFO("[08] En4: [%d  ]", R_En4);

                  //     ROS_INFO("[09] State_Claw1: [%d  ]", R_C1);

                  //     ROS_INFO("[10] State_Claw2: [%d  ]", R_C2);  							
								
                  //     ROS_INFO("[11] Voltage: [%.2f V]", (float)Voltage/100); // 电池电压
                  //     ROS_INFO("-----------------------"); 
                  //     ROS_INFO("a: [%d ]",   a);
                  //     ROS_INFO("b: [%d ]",   b);
									// 		ROS_INFO("a/b: [%.2f ]",   (float)a/(float)b);
                  //     if(b>5000)b=b/10,a=a/10;														
                  //    }					 

        loop_rate.sleep();//循环延时时间
   }


   //关闭串口
   sp.close();  

    return 0;
}

//************************发送数据**************************// 

void send_data(void)
{
    uint8_t tbuf[27];

    tbuf[26]=0;  //校验位置零
    tbuf[0]=0XAA;   //帧头
    tbuf[1]=0XAA;   //帧头
    tbuf[2]=0XF1;    //功能字
    tbuf[3]=22;    //数据长度
              tbuf[4]=	S_H1>>0;// 
              tbuf[5]=	S_H1>>8;//
              tbuf[6]=	S_H1>>16;//
              tbuf[7]=	S_H1>>24;//

              tbuf[8]=	S_L1>>0;// 
              tbuf[9]=	S_L1>>8;//
              tbuf[10]=	S_L1>>16;//
              tbuf[11]=	S_L1>>24;//

              tbuf[12]=	S_H2>>0;// 
              tbuf[13]=	S_H2>>8;//
              tbuf[14]=	S_H2>>16;//
              tbuf[15]=	S_H2>>24;//

              tbuf[16]=	S_L2>>0;// 
              tbuf[17]=	S_L2>>8;//
              tbuf[18]=	S_L2>>16;//
              tbuf[19]=	S_L2>>24;//
							
              tbuf[20]=	S_En1;//
              tbuf[21]=	S_En2;//
							tbuf[22]=	S_En3;//
              tbuf[23]=	S_En4;//
							
              tbuf[24]=	S_C1;//
              tbuf[25]=	S_C2;//
							
    for(uint8_t i=0;i<26;i++)tbuf[26]+=tbuf[i];//计算校验和 
  try
  {
    sp.write(tbuf, 27);//发送数据下位机(数组，字节数)

  }
  catch (serial::IOException& e)   
  {
    ROS_ERROR_STREAM("Unable to send data through serial port"); //如果发送数据失败，打印错误信息
  }
}  



void initialize_arms()
{
    // 初始化所有手臂參數
    S_H1 = 0; // 手臂1的高度
    S_L1 = 0; // 手臂1的長度
    S_H2 = 0; // 手臂2的高度
    S_L2 = 0; // 手臂2的長度
    S_En1 = 1; // 啟用電機1
    S_En2 = 1; // 啟用電機2
    S_En3 = 1; // 啟用電機3
    S_En4 = 1; // 啟用電機4
    S_C1 = 0;  // 爪子1張開
    S_C2 = 0;  // 爪子2張開

    send_data(); // 發送初始化數據到下位機
}




//************************接收并处理来自下位机的数据**************************// 
//************************接收并处理来自下位机的数据**************************// 
//************************接收并处理来自下位机的数据**************************// 
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
       size_t n = sp.available();//获取缓冲区内的字节数
       a++;
         if(n>0)  
               {		   
                 uint8_t buffer[30];uint8_t buf[30];
                 
                 if(n>=62){
                   while(n){n = sp.available();if(n>=62)sp.read(buf, 30);else {break;}}//砍掉旧缓存，获取最新数据  ,预防堵塞                 
                 }                 
                 if(n>=31 && n<62){
                     for(uint8_t i=0;i<n;i++){
                         if(buffer[0]!=0XAA)sp.read(buffer, 1);
                         else {break;} 
                     }//逐个读字节，读到帧头跳出
                  }                    
                 if(buffer[0]==0XAA)//
                  {
                   sp.read(buffer, 30);//                 
                   if(buffer[0]==0XAA && buffer[1]==0XF1)
                      {              
                       uint8_t sum=0; 
	               for(uint8_t j=0;j<29;j++)sum+=buffer[j];    //计算校验和	
                       if(buffer[29] == (uint8_t)(sum+buffer[0]))
                          {b++;	Flag_OK=1;
 						                   R_H1  = (int32_t)((buffer[3]<<0) |(buffer[4]<<8) |(buffer[5]<<16) |(buffer[6]<<24));//单位毫米
														   R_L1  = (int32_t)((buffer[7]<<0) |(buffer[8]<<8) |(buffer[9]<<16) |(buffer[10]<<24));//单位毫米
															 R_H2  = (int32_t)((buffer[11]<<0)|(buffer[12]<<8)|(buffer[13]<<16)|(buffer[14]<<24));//单位毫米
															 R_L2  = (int32_t)((buffer[15]<<0)|(buffer[16]<<8)|(buffer[17]<<16)|(buffer[18]<<24));//单位毫米
										
										           R_En1 = buffer[19];
										           R_En2 = buffer[20]; 
										           R_En3 = buffer[21]; 
										           R_En4 = buffer[22];
 										
						                   R_C1    = buffer[23]; 
														   R_C2    = buffer[24];
														
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

           std::cout<< "[03] Current_Height_2:" << (int)R_H2 <<"[mm]"<<std::endl;
           std::cout<< "[04] Current_length_2:" << (int)R_L2 <<"[mm]"<<std::endl;

           std::cout<< "[05] (Height_1)En1:" <<  (int)R_En1  <<std::endl;
           std::cout<< "[06] (length_1)En2:" <<  (int)R_En2  <<std::endl;

           std::cout<< "[07] (Height_2)En3:" <<   (int)R_En3  <<std::endl;
           std::cout<< "[08] (length_2)En4:" <<  (int)R_En4  <<std::endl;
					
           std::cout<< "[09] State_Claw1:" <<   (int)R_C1  <<std::endl;
           std::cout<< "[10] State_Claw2:" <<  (int)R_C2  <<std::endl;
					
           std::cout<< "[11] Voltage:" << (float)Voltage/100 <<std::endl;//电池电压
                      								 
           std::cout<< "[12] 主循环频数a:" << (uint16_t)a <<std::endl;
           std::cout<< "[13] 有效接收数b:" << (uint16_t)b <<std::endl;                                        
           std::cout<< "[14] a/b:" <<  (float)a/b <<std::endl;
           if(b>5000)b=b/10,a=a/10;
         
           std::cout<< "-----------------------" <<std::endl;           														
                     }                         
}
