/*
*	@Author: PingLin Zhang
*	@Date:	 2020.04.13
*	@Brief:  Serial
*/
#ifndef  _USART_H
#define  _USART_H

//串口相关的头文件
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
//宏定义
#define FALSE  -1
#define TRUE   0

typedef struct vision_te
{
    float enemy_angle_360;
    float x_filter;
    float vx_filter;
    float ax_filter;
    float y_filter;
    float vy_filter;
    float ay_filter;
    float pre_yaw;
    float pre_pitch;
}vision_test;

typedef struct vision_pridict
{
    float yaw_now;//yaw
    float pitch_now;//pitch
    float Bullet_speed;//单位 m/s
    float position;//当前位置
    float move_v;  //
}vision_pr;

/**
* @brief
* @param
*/
typedef struct vision_get
{
    uint8_t  Enemy_color;

    int16_t infantry3_HP;
    int16_t infantry4_HP;

    vision_pr little_vision;
}vision_re, *Parse_vision_re;


typedef struct uwb_coord
{
  public:
  float x_coord;
  float y_coord;
}uwb,P_uwb;


class Serial
{
public:
    Serial();
    ~Serial();
  
    ros::NodeHandle nh_;
    ros::Publisher serial_pub;
    ros::Publisher imu_pub;

    /**
    * @brief Serial main function
    * @param fire: Whether to fire
    * @param find: whether to find armor
    */
    int serialMode(double yaw, double pitch, double distance, char armor_change, bool find, Parse_vision_re vision_get_main);

    int serialMode();
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

    /**
    * @brief 在串口读取到的数据中提取出一个数据包的数据段,转存到infoArray中，供后续解析为CSInfoStructure.
    * @param infoArray 存放一个完整的数据包
    * @param packages 从串口读取到的包含数据包的数据
    * @param sizepackages 从串口读取到的字节数（packages的大小）
    */
      bool get_one_in_packages(uint8_t * infoArray, uint8_t * packages);

    /**
     * @brief  把存有一个数据段的数组解析为一个数据结构体,结果存到参数2对应的地址
     * @param  infoArray 存有一个数据段的uint8类型的数组
     * @param  infoStrc 从串口读取到的字节数（packages的大小）
     * @retval 无
     */
     void buff_to_vision_receive(uint8_t * uart0_recv_buf,Parse_vision_re receive_buf);

};

#endif


