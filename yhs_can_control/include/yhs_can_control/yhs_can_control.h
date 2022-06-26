#ifndef __PIDCONTROL_NODE_H__
#define __PIDCONTROL_NODE_H__



#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "yhs_can_msgs/ctrl_cmd.h"
#include "yhs_can_msgs/io_cmd.h"
#include "yhs_can_msgs/ctrl_fb.h"
#include "yhs_can_msgs/lr_wheel_fb.h"
#include "yhs_can_msgs/rr_wheel_fb.h"
#include "yhs_can_msgs/io_fb.h"
#include "yhs_can_msgs/odo_fb.h"
#include "yhs_can_msgs/bms_Infor_fb.h"
#include "yhs_can_msgs/bms_flag_Infor_fb.h"
#include "yhs_can_msgs/Drive_MCUEcoder_fb.h"
#include "yhs_can_msgs/Veh_Diag_fb.h"
#include "yhs_can_msgs/hw.h"
#include "yhs_can_msgs/state.h"


#include "controlcan.h"




#include<stdio.h>      /*标准输入输出定义*/
#include<stdlib.h>     /*标准函数库定义*/
#include<unistd.h>     /*Unix 标准函数定义*/
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>      /*文件控制定义*/
#include<termios.h>    /*PPSIX 终端控制定义*/
#include<errno.h>      /*错误号定义*/
#include<string.h>
#include<time.h>
#include<stdint.h>
#include<iostream>

#include<ros/ros.h>
#include "yhs_can_msgs/serial.h"
#include "yhs_can_msgs/imu.h"
#include <tf/transform_broadcaster.h>    



namespace yhs_tool {
class CanControl
{
public:
	CanControl();
	~CanControl();
	
	void run();
private:


	ros::NodeHandle nh_;

	ros::Publisher ctrl_fb_pub_;
	ros::Publisher lr_wheel_fb_pub_;
	ros::Publisher rr_wheel_fb_pub_;
	ros::Publisher io_fb_pub_;
	ros::Publisher odo_fb_pub_;
  ros::Publisher odom_pub_;
	ros::Publisher bms_Infor_fb_pub_;
	ros::Publisher bms_flag_Infor_fb_pub_;
	ros::Publisher Drive_MCUEcoder_fb_pub_;
	ros::Publisher Veh_Diag_fb_pub_;

	ros::Subscriber ctrl_cmd_sub_;
	ros::Subscriber io_cmd_sub_;



	boost::mutex cmd_mutex_io_;

	boost::mutex cmd_mutex_vel_;

	boost::mutex cmd_mutex_;

	unsigned char sendData_u_io_[8] = {0};
	unsigned char sendData_u_vel_[8] = {0};

	unsigned int canID_io_ = 0;
	unsigned int canID_vel_ = 0;

	VCI_CAN_OBJ send[1];


	void io_cmdCallBack(const yhs_can_msgs::io_cmd msg);
	void ctrl_cmdCallBack(const yhs_can_msgs::ctrl_cmd msg);


	void recvData();
	void sendData();

};



class Serial
{
public:
    Serial();
    ~Serial();
  
    ros::NodeHandle nh_;
    ros::Publisher serial_pub;
    ros::Publisher imu_pub;

    ros::Subscriber car_state_sub;

    int car_state;
    bool init_imu_pose=false;
    /**
    * @brief Serial main function
    * @param fire: Whether to fire
    * @param find: whether to find armor
    */
    void uwb_serialMode();
    void imu_serialMode();
    void hw_serialMode();
    void laser_serialMode();


    void car_state_callback(const yhs_can_msgs::state msg);
    double calc_distance(double x1,double y1,double x2,double y2);
    void serialMode();
    /*******************************************************************
    *名称：             UART0_Open
    *功能：             打开串口并返回串口设备文件描述
    *入口参数：         fd      文件描述符
                        port    串口号(ttyTHS0,ttyTHS1,ttyTHS2)
    *出口参数：正确返回为1，错误返回为0
    *******************************************************************/
      int UART0_Open(int fd,char*port);

    /*******************************************************************
    *名称：             UART0_Set
    *功能：             设置串口数据位，停止位和效验位
    *入口参数：         fd          串口文件描述符
    *                   speed       串口速度
    *                   flow_ctrl   数据流控制
    *                   databits    数据位   取值为 7 或者8
    *                   stopbits    停止位   取值为 1 或者2
    *                   parity      效验类型 取值为N,E,O,,S
    *出口参数：正确返回为1，错误返回为0
    *******************************************************************/
      int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity);

    /*******************************************************************
    *名称：                UART0_Init()
    *功能：                串口初始化
    *入口参数：            fd         文件描述符
    *                      speed      串口速度
    *                      flow_ctrl  数据流控制
    *                      databits   数据位   取值为 7 或者8
    *                      stopbits   停止位   取值为 1 或者2
    *                      parity     效验类型 取值为N,E,O,,S
    *
    *出口参数：正确返回为1，错误返回为0
    *******************************************************************/
      int UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity);

    /*******************************************************************
    * 名称：            UART0_Recv
    * 功能：            接收串口数据
    * 入口参数：        fd         文件描述符
    *                   rcv_buf    接收串口中数据存入rcv_buf缓冲区中
    *                   data_len   一帧数据的长度
    * 出口参数：        正确返回为1，错误返回为0
    *******************************************************************/
      int UART0_Recv(int fd, char *rcv_buf,int data_len);

    /********************************************************************
    * 名称：            UART0_Send
    * 功能：            发送数据
    * 入口参数：        fd           文件描述符
    *                   send_buf     存放串口发送数据
    *                   data_len     一帧数据的个数
    * 出口参数：        正确返回为1，错误返回为0
    *******************************************************************/
      int UART0_Send(int fd, uint8_t *send_buf,int data_len);
private:
    friend class CanControl;

};

}


#endif

