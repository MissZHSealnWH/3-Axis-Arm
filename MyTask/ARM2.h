#ifndef _ARM2_H_
#define _ARM2_H_

#include "Task_Init.h"
#include "semphr.h"

#define MAX_VEL  0.2f
#define MAX_ACC  0.5f
#define STEP_SIZE 0.03f
#define M_PI 3.1415926535f

#define PULSE_PER_REVOLUTION   (11 * 4)
#define REDUCTION_RATIO        56
#define RAD2ENC_FACTOR_JOINT  ((PULSE_PER_REVOLUTION * REDUCTION_RATIO) / (2.0f * M_PI))

typedef struct
{
	PID2 pid;
} motor_PID;

typedef struct
{
	int32_t Exp_encoder;
	float   Exp_speed;
} Param;

typedef struct
{
	int32_t T_encoder;
	float   T_speed;
}TEXT_Param;

extern SemaphoreHandle_t g_EncoderMutex;

#endif
