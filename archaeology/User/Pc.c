#include "PC.h"
#include "Bucket.h"
#include "USER_CAN.h"
#include "usart.h"
#include "tim.h"
#include "Crc.h"

void PC_Data_Init(uint8_t* rxData);
void PC_Request_Handle(uint8_t* rxData);

uint8_t ack[]={0xaa,0x3f,0x3f};//Ӧ��֡
uint8_t pc_usart_rx_buff[15];//����pc���ڻ�����
uint8_t pc_tx_buff[10]={0x55};//���ʹ��ڻ�����
uint16_t cmpValue=0;//����Ƚ�ֵ
uint16_t waitime=10000-1;
bool bucketFinish=true;
bool initFlag=false;

extern uint8_t pc_tx_buff[10];

void PC_Init()
{
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);	
}

/************PC����3****************/
//PC����3�жϻص�
void PC_USART3_IRQHandler()
{
	static uint16_t rxCnt=0;
	//����ÿһ���ֽ�
	if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE))
	{
		if(rxCnt<sizeof(pc_usart_rx_buff))
			pc_usart_rx_buff[rxCnt++] = huart3.Instance->DR;
		__HAL_UART_CLEAR_FLAG(&huart3,UART_FLAG_RXNE);
	}
	//����֡��������
  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))
  {
    /* clear idle it flag avoid idle interrupt all the time */
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    if (rxCnt == PC_INIT_LEN)
    {
      PC_Data_Init(pc_usart_rx_buff);//������յ���PC����
    }
		else if(rxCnt == PC_DATA_LEN)
		{
			PC_Request_Handle(pc_usart_rx_buff);//������յ���PC����
		}
		rxCnt = 0;
  }
}

//����pc��ʼ������
void PC_Data_Init(uint8_t* rxData)
{
	if(CRC_Calculate(rxData,PC_INIT_LEN))
		return;
	uint8_t txBuff[8]={0};
	switch(rxData[0])
	{
		case 0x55:
			cmpValue=((rxData[1]<<8)|rxData[2]);
			waitime=((rxData[3]<<8)|rxData[4]);
			txBuff[0]=rxData[5];
			txBuff[1]=rxData[6];
			USER_CAN_SendData(&hcan1,0x200,txBuff);
			initFlag=true;
		break;
	}
	HAL_UART_Transmit_DMA(&huart3,ack,sizeof(ack));
}

//����pc����ָ��
void PC_Request_Handle(uint8_t* rxData)
{
	if(CRC_Calculate(rxData,PC_DATA_LEN))
		return;
	uint8_t txBuff[8]={0};
	if(initFlag)
	{
		if(rxData[0]==0x50)
		{
			switch(rxData[1])
			{
				case 1:
					HAL_UART_Transmit_DMA(&huart3,pc_tx_buff,5);
				break;
				case 2:
					if(bucketFinish)
					{
						bucketFinish=false;
						Bucket_Up();
						HAL_TIM_Base_Start_IT(&htim6);//������ʱ���ж�
						HAL_UART_Transmit_DMA(&huart3,ack,sizeof(ack));
					}
				break;
				case 3:
				case 4:
					txBuff[0]=rxData[1]-2;
					USER_CAN_SendData(&hcan1,0x201,txBuff);
				break;
			}
		}
	}
}
