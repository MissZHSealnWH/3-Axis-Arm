#ifndef _ARM2_H_
#define _ARM2_H_

#include "Task_Init.h"

// 运动参数
#define MAX_VEL  0.2f        // 末端最大速度 m/s
#define MAX_ACC  0.5f        // 末端最大加速度 m/s2
#define STEP_SIZE 0.03f      // 每一次前进/后退移动的距离 (米)
#define M_PI 3.1415926535f


#define PULSE_PER_REVOLUTION   (11 * 4)         // 44
#define REDUCTION_RATIO        56               // 减速比
#define RAD2ENC_FACTOR_JOINT  ((PULSE_PER_REVOLUTION * REDUCTION_RATIO) / (2.0f * M_PI))

typedef struct
{
PID2 pid;
	
}motor_PID;

typedef struct
{
	int16_t Exp_encoder;
	float   Exp_speed;
}Param;

#endif
