#ifndef _TASK_INIT_H_
#define _TASK_INIT_H_

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "queue.h"
#include "math.h"




extern TaskHandle_t ARM2_Handle;
extern TaskHandle_t Analysis_Handle;
extern TaskHandle_t ZDT_Parse_Run_Handle;

void Task_Init(void);

void ARM2(void *pvParameters);
void Analysis(void *pvParameters);
void ZDT_Parse_Run(void *pvParameters);
#endif


