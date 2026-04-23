#ifndef _TASK_INIT_H_
#define _TASK_INIT_H_

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "queue.h"
#include "PID_old.h"
#include "math.h"
#include "app_motor_usart.h"
#include "bsp_motor_usart.h"






extern TaskHandle_t ARM_Handle;
extern TaskHandle_t ARM2_Handle;


void Task_Init(void);

// ÈÎÎñº¯Êý
void ARM(void *pvParameters);
void ARM2(void *pvParameters);
#endif


