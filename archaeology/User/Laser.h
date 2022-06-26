#ifndef __LASER_H__
#define __LASER_H__

#include "main.h"

//#define MODBUS 
#define LASER_DATA_LEN 7

extern uint16_t laserValue[4];

void Laser_Init(void);
void Laser_USART2_IRQHandler(void);
void Laser_USART6_IRQHandler(void);
void Laser_USART7_IRQHandler(void);
void Laser_USART8_IRQHandler(void);

#endif
