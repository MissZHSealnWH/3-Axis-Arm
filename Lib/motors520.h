#ifndef __BSP_MOTOR_USART_H_
#define __BSP_MOTOR_USART_H_

#include "stdio.h"
#include "Task_Init.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>


#include "motor520.h"


void Send_Motor_Frame(uint8_t *buf, uint16_t len);
void Send_Motor_Array(uint8_t *pData, uint16_t Length);
void Motor_Usart_Init(void);
static void start_tx_byte(void);


#endif

