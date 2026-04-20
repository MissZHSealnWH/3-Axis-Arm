#ifndef _ARM_H_
#define _ARM_H_

#include "Task_Init.h"

typedef struct
{
	PID2 vel_pid;
	PID2 pos_pid;
}TIM_PID;

typedef struct
{
	float Exp_pos;
	float Exp_vel;
}TIM_Exp_Prama;

#endif
