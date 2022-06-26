#ifndef __USER_UART_H__
#define __USER_UART_H__

#include "main.h"
#include <stdbool.h>

//define MODBUS
#define LASER_DATA_LEN (7u)
#define IMU_DATA_LEN (11u)

extern uint16_t cmpValue;
extern uint16_t laserValue[4];
extern uint16_t waitime;

void USER_UART_Init(void);
void Laser_USART2_IRQHandler(void);
void Laser_USART6_IRQHandler(void);
void Laser_USART7_IRQHandler(void);
void Laser_USART8_IRQHandler(void);
void IMU_USART3_IRQHandler(void);

#endif
