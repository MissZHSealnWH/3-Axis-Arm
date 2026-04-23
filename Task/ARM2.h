#ifndef _ARM2_H_
#define _ARM2_H_

#include "Task_Init.h"

//typedef struct
//{
//	PID2 vel_pid;
//	PID2 pos_pid;
//}TIM_PID;

//typedef struct
//{
//	float Exp_pos;
//	float Exp_vel;
//}TIM_Exp_Prama;

typedef struct
{
PID2 pid;
	
}motor_PID;

typedef struct
{
	int16_t Exp_encoder;
}Param;

#endif
