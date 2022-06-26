#ifndef __PC_H__
#define __PC_H__

#include "main.h"
#include <stdbool.h>

#define PC_DATA_LEN 4
#define PC_INIT_LEN 9

extern uint16_t cmpValue;
extern uint8_t pc_tx_buff[10];
extern uint16_t waitime;
extern bool bucketFinish;

void PC_Init(void);
void PC_USART3_IRQHandler(void);

#endif
