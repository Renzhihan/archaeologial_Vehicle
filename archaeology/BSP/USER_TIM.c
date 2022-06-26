#include "USER_TIM.h"
#include "Laser.h"
#include "PC.h"
#include "Bucket.h"
#include "Crc.h"
#include "usart.h"
#include "tim.h"
#include "stdbool.h"

uint8_t getCmd[]={0x50,0x03,0x00,0x34,0x00,0x01,0xc8,0x45};

void USER_TIM_Init()
{
	__HAL_TIM_CLEAR_FLAG(&htim6,TIM_FLAG_UPDATE);
	HAL_TIM_Base_Start_IT(&htim7);//开启定时器中断
}

//定时器中断
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==htim7.Instance)
	{
		bool isPit[4]={laserValue[0]>(cmpValue),laserValue[1]>cmpValue,laserValue[2]>cmpValue,laserValue[3]>(cmpValue)};
		if((isPit[0]&&!isPit[1]&&!isPit[2]&&isPit[3])||(!isPit[0]&&!isPit[1]&&!isPit[2]&&!isPit[3]))
		{
			pc_tx_buff[1]=0;
			pc_tx_buff[2]=0;
		}
		else if((isPit[0]&&isPit[1]&&!isPit[2]&&isPit[3]))
		{
			pc_tx_buff[1]=1;
			pc_tx_buff[2]=0;
		}
		else if((isPit[0]&&isPit[1]&&!isPit[2]&&!isPit[3]))
		{
			pc_tx_buff[1]=2;
			pc_tx_buff[2]=0;
		}
		else if((isPit[0]&&!isPit[1]&&isPit[2]&&isPit[3]))
		{
			pc_tx_buff[1]=0;
			pc_tx_buff[2]=1;
		}
		else if((!isPit[0]&&!isPit[1]&&isPit[2]&&isPit[3]))
		{
			pc_tx_buff[1]=0;
			pc_tx_buff[2]=2;
		}
		uint16_t crc=CRC_Calculate(pc_tx_buff,3);//进行crc计算
		pc_tx_buff[3]=crc>>8;
		pc_tx_buff[4]=crc;
			
		HAL_UART_Transmit_DMA(&huart2,getCmd,sizeof(getCmd));//发送回传激光数据命令
		HAL_UART_Transmit_DMA(&huart6,getCmd,sizeof(getCmd));
		HAL_UART_Transmit_DMA(&huart7,getCmd,sizeof(getCmd));
		HAL_UART_Transmit_DMA(&huart8,getCmd,sizeof(getCmd));
	}
	else if(htim->Instance==htim6.Instance)
	{
		static uint8_t state=1;
		static uint8_t bucketState_tx_buff[10]={0xff};
		HAL_TIM_Base_Stop_IT(&htim6);
		__HAL_TIM_SetCounter(&htim6,0);
		switch(state)
		{
			case 1:
				bucketState_tx_buff[1]=1;
				bucketState_tx_buff[2]=0x81;
				bucketState_tx_buff[3]=0x80;
				__HAL_TIM_SetAutoreload(&htim6,waitime);
				state++;
				Bucket_Stop();
				HAL_TIM_Base_Start_IT(&htim6);
				HAL_UART_Transmit_DMA(&huart3,bucketState_tx_buff,4);
			break;
			case 2:
				bucketState_tx_buff[1]=2;
				bucketState_tx_buff[2]=0xc1;
				bucketState_tx_buff[3]=0x81;
				__HAL_TIM_SetAutoreload(&htim6,30000-1);
				state++;
				Bucket_Down();
				HAL_TIM_Base_Start_IT(&htim6);
				HAL_UART_Transmit_DMA(&huart3,bucketState_tx_buff,4);
			break;
			case 3:
				bucketState_tx_buff[1]=3;
				bucketState_tx_buff[2]=0x00;
				bucketState_tx_buff[3]=0x41;
				state=1;
				Bucket_Stop();
				bucketFinish=true;
				HAL_UART_Transmit_DMA(&huart3,bucketState_tx_buff,4);
			break;
		}
	}
}
