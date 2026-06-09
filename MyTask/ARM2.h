#ifndef _ARM2_H_
#define _ARM2_H_

#include "Task_Init.h"
#include "semphr.h"
#include "ZDT_Emm.h"

#define MAX_VEL  0.15f
#define MAX_ACC  0.3f
#define STEP_SIZE 0.003f  // 每cmd单位对应3mm
#define M_PI 3.1416f

#define DEG_TO_RAD(deg) ((deg) * (M_PI / 180.0f))
#define RAD_TO_DEG(rad) ((rad) * (180.0f / M_PI))



#define JOINT1_MIN 0
#define JOINT1_MAX 3.1416f

#define JOINT2_MIN 0
#define JOINT2_MAX 3.1416f


typedef struct{
    ZDT_Emm_t *zdt;
    float_t exp_pos;
    float_t exp_vel;
    float_t exp_acc;
    float_t cur_pos;
    float_t cur_vel;
    float_t cur_acc;
    struct{
    float_t kp;
    float_t exp_pos;
    float_t exp_vel;
    float_t exp_acc;
    float_t cur_pos;
    float_t cur_vel;
    float_t cur_acc;
    } zdt_st;
    float_t dir;  //���ֱ�+���򣨹ؽڵĽǶ�->����Ľǣ��ؽڵĽǶ�*dir=����Ľ�
    float_t kx;  //�������(�˵ĽǶ�->�ؽڵĽǶ�)   �˵ĽǶ�*kx=�ؽڵĽǶ�
    float_t foot_state;
} Joint_t;


static void ZDT_Parse_One(uint8_t buf_index, const uint8_t *data, uint16_t size);

extern SemaphoreHandle_t g_EncoderMutex;

#endif
