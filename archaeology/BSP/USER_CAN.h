#ifndef _USER_CAN_H_
#define _USER_CAN_H_

#include "main.h"
#include "can.h"

/****接口函数声明****/
void USER_CAN_Init(void);
//CAN发送数据
uint8_t USER_CAN_SendData(CAN_HandleTypeDef* hcan,uint32_t StdId,uint8_t data[8]);

#endif
