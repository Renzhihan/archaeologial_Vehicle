#include "USER_CAN.h"
#include "USER_UART.h"
#include "Crc.h"

extern uint8_t laser_tx_buff[8];
extern uint8_t imu_usart_rx_buff[20];

//CAN发送数据
uint8_t USER_CAN_SendData(CAN_HandleTypeDef* hcan,uint32_t StdId,uint8_t data[8]);

//can接收结束中断
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef header;
  uint8_t rx_data[8];
	
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, rx_data);
	
	uint8_t tx_data[8]={0};
	
	if(hcan==&hcan1)
	switch(header.StdId)
	{
		case 0x200:
			cmpValue=(rx_data[0]<<8)|rx_data[1];
		break;
		case 0x201:
			if(rx_data[0]==1)
			{
				for(uint8_t i=0;i<4;i++)
					tx_data[i]=laser_tx_buff[i+1];
				USER_CAN_SendData(&hcan1,0x210,tx_data);
			}
			else if(rx_data[0]==2)
			{
				for(uint8_t i=0;i<6;i++)
					tx_data[i]=imu_usart_rx_buff[i+2];
				//进行和校验
				uint8_t sumchk=0;
				uint16_t crc=0;
				for(uint8_t i=0;i<10;i++)
					sumchk+=imu_usart_rx_buff[i];
				if(sumchk==imu_usart_rx_buff[10])
				{
					crc=CRC_Calculate(tx_data,6);
					tx_data[6]=crc>>8;
					tx_data[7]=crc;
				}
				USER_CAN_SendData(&hcan1,0x211,tx_data);
			}
		break;
	}
}

//can初始化，在main的while(1)前调用
void USER_CAN_Init()
{
	uint8_t errCnt=0;//错误计数
	//CAN1过滤器初始化
	CAN_FilterTypeDef Can1_Filter;
	Can1_Filter.FilterActivation = ENABLE;
  Can1_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
  Can1_Filter.FilterScale = CAN_FILTERSCALE_32BIT;
  Can1_Filter.FilterIdHigh = 0x0000;
  Can1_Filter.FilterIdLow = 0x0000;
  Can1_Filter.FilterMaskIdHigh = 0x0000;
  Can1_Filter.FilterMaskIdLow = 0x0000;
  Can1_Filter.FilterBank = 0;
  Can1_Filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	errCnt+=HAL_CAN_ConfigFilter(&hcan1, &Can1_Filter);
	//开启CAN1
	errCnt+=HAL_CAN_Start(&hcan1);
	errCnt+=HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

//CAN发送数据
uint8_t USER_CAN_SendData(CAN_HandleTypeDef* hcan,uint32_t StdId,uint8_t data[8])
{
	CAN_TxHeaderTypeDef tx_header;
	
	tx_header.StdId = StdId;
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;
	
	uint8_t retVal=HAL_CAN_AddTxMessage(hcan, &tx_header, data, (uint32_t*)CAN_TX_MAILBOX0);
		
	return retVal;
}

