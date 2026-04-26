#ifndef _ARM2_H_
#define _ARM2_H_

#include "Task_Init.h"


typedef struct
{
PID2 pid;
	
}motor_PID;

typedef struct
{
	int16_t Exp_encoder;
	float Exp_speed;
}Param;

#endif
