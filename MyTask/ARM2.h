#ifndef _ARM2_H_
#define _ARM2_H_

#include "Task_Init.h"
#include "semphr.h"

#define MAX_VEL  0.2f
#define MAX_ACC  0.5f
#define STEP_SIZE 0.05f
#define M_PI 3.1415926535f

#define PULSE_PER_REVOLUTION   (11 * 4)
#define REDUCTION_RATIO        56
#define BASE_GEAR_RATIO        2.0f           // 底座额外齿轮比

#define RAD2ENC_FACTOR_JOINT   ((PULSE_PER_REVOLUTION * REDUCTION_RATIO) / (2.0f * M_PI))
#define RAD2ENC_FACTOR_JOINT0  ((PULSE_PER_REVOLUTION * REDUCTION_RATIO * BASE_GEAR_RATIO) / (2.0f * M_PI))

#define JOINT1_MIN 0
#define JOINT1_MAX 3.1416f

#define JOINT2_MIN  0
#define JOINT2_MAX  3.1416f

#define TAU_TO_SPEED_FF  0.01f   // 例：0.01 → 每 N·m 对应 1% 速度量程

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
