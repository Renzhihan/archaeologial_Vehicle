#include "USER_CAN.h"
#include "usart.h"
#include "Crc.h"

uint8_t backLaser[10]={0};
uint8_t imu[10]={0};

//can接收结束中断
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef header;
  uint8_t rx_data[8];
	
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, rx_data);
	
	if(hcan==&hcan1)
	switch(header.StdId)
	{
		case 0x210:
			backLaser[0]=0x54;
			for(uint8_t i=0;i<4;i++)
				backLaser[i+1]=rx_data[i];
			HAL_UART_Transmit_DMA(&huart3,backLaser,5);
		break;
		case 0x211:
			imu[0]=0x53;
			for(uint8_t i=0;i<8;i++)
				imu[i+1]=rx_data[i];
			HAL_UART_Transmit_DMA(&huart3,imu,9);
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

