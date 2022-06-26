#include "USER_UART.h"
#include "usart.h"
#include "Crc.h"
#include <stdlib.h>
#include <string.h>

void Laser_GetData(uint8_t* data,uint16_t* value);

uint8_t laser_usart_rx_buff[4][40];//接收激光串口缓冲区
uint16_t laserValue[4];//激光测距值
uint16_t cmpValue=0;//激光比较值
uint8_t imu_usart_rx_buff[20];

extern uint8_t pc_tx_buff[10];

void USER_UART_Init()
{
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart6,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE);	
	__HAL_UART_ENABLE_IT(&huart7,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart7,UART_IT_IDLE);	
	__HAL_UART_ENABLE_IT(&huart8,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart8,UART_IT_IDLE);	
}

/**************************激光串口****************************/
//激光串口2中断回调
void Laser_USART2_IRQHandler()
{
	static uint16_t rxCnt=0;
	if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE))
	{
		laser_usart_rx_buff[0][rxCnt++] = huart2.Instance->DR;
	}
  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
  {
    /* clear idle it flag avoid idle interrupt all the time */
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);
    if (rxCnt == LASER_DATA_LEN)
    {
      Laser_GetData(laser_usart_rx_buff[0],&laserValue[0]);//解码接收到的数据
    }
		rxCnt = 0;
  }
}

//激光串口6中断回调
void Laser_USART6_IRQHandler()
{
	static uint16_t rxCnt=0;
	if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE))
	{
		laser_usart_rx_buff[1][rxCnt++] = huart6.Instance->DR;
	}
  if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))
  {
    /* clear idle it flag avoid idle interrupt all the time */
    __HAL_UART_CLEAR_IDLEFLAG(&huart6);
		if (rxCnt == LASER_DATA_LEN)
    {
      Laser_GetData(laser_usart_rx_buff[1],&laserValue[1]);//解码接收到的数据
    }
		rxCnt = 0;
  }
}

//激光串口7中断回调
void Laser_USART7_IRQHandler()
{
	static uint16_t rxCnt=0;
	if (__HAL_UART_GET_FLAG(&huart7, UART_FLAG_RXNE))
	{
		laser_usart_rx_buff[2][rxCnt++] = huart7.Instance->DR;
	}
  if (__HAL_UART_GET_FLAG(&huart7, UART_FLAG_IDLE))
  {
    /* clear idle it flag avoid idle interrupt all the time */
    __HAL_UART_CLEAR_IDLEFLAG(&huart7);
		if (rxCnt == LASER_DATA_LEN)
    {
      Laser_GetData(laser_usart_rx_buff[2],&laserValue[2]);//解码接收到的数据
    }
		rxCnt = 0;
  }
}

//激光串口8中断回调
void Laser_USART8_IRQHandler()
{
	static uint16_t rxCnt=0;
	if (__HAL_UART_GET_FLAG(&huart8, UART_FLAG_RXNE))
	{
		laser_usart_rx_buff[3][rxCnt++] = huart8.Instance->DR;
	}
  if (__HAL_UART_GET_FLAG(&huart8, UART_FLAG_IDLE))
  {
    /* clear idle it flag avoid idle interrupt all the time */
    __HAL_UART_CLEAR_IDLEFLAG(&huart8);
		if (rxCnt == LASER_DATA_LEN)
    {
      Laser_GetData(laser_usart_rx_buff[3],&laserValue[3]);//解码接收到的数据
    }
		rxCnt = 0;
  }
}

//IMU串口3中断回调
void IMU_USART3_IRQHandler()
{
	static uint16_t rxCnt=0;
	if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE))
	{
		imu_usart_rx_buff[rxCnt++] = huart3.Instance->DR;
	}
  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))
  {
    /* clear idle it flag avoid idle interrupt all the time */
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
		if (rxCnt == IMU_DATA_LEN)
    {
			
    }
		rxCnt = 0;
  }
}

//解析激光传感器数据
void Laser_GetData(uint8_t* data,uint16_t* value)
{
	#ifdef MODBUS
	if(CRC_Calculate(data,7))
		return;
	*value=((data[3]<<8)|data[4]);
	#else
	char* temp;
	temp=strstr((char*)data,"d:  ");
	if(temp!=NULL)
		*value=atoi(temp+4);
	#endif
}
