/**
 * @file PWMmotor.c
 * @editor CaoLiangRui
 * @Data 2026.4.20
 * @brief 霍尔亚博520直流电机驱动器控制实现
 */

#include "PWMmotor.h"
#include "tim.h"


/** @brief IN1引脚的定时器通道 */
#define IN1_CH TIM_CHANNEL_1
/** @brief IN2引脚的定时器通道 */
#define IN2_CH TIM_CHANNEL_2

/** @brief 最大速度值 */
#define MAX_SPEED 100


/** @brief 当前衰减模式 */
static DecayMode currentDecayMode = SLOW_DECAY;

/**
 * @brief 设置IN1引脚的PWM占空比
 * @param duty 占空比值
 */
static inline void __SetIn1PWM(TIM_HandleTypeDef *htim, uint8_t duty)
{
    __HAL_TIM_SET_COMPARE(htim, IN1_CH, duty);
}

/**
 * @brief 设置IN2引脚的PWM占空比
 * @param duty 占空比值
 */
static inline void __SetIn2PWM(TIM_HandleTypeDef *htim, uint8_t duty)
{
    __HAL_TIM_SET_COMPARE(htim, IN2_CH, duty);
}

/**
 * @brief 初始化
 */
void motor520_Init(TIM_HandleTypeDef *htim)
{
  HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

/**
 * @brief 设置衰减模式
 * @param mode 衰减模式
 */
void motor520_SetDecayMode(DecayMode mode)
{
    currentDecayMode = mode;
}

/**
 * @brief 控制电机前进
 * @param speed 速度值（0-100）
 */
void motor520_Forward(TIM_HandleTypeDef *htim, uint8_t speed)
{
    if (speed > MAX_SPEED)
        speed = MAX_SPEED;
    
    if (currentDecayMode == FAST_DECAY) {
        __SetIn1PWM(htim, speed);
        __SetIn2PWM(htim, 0);
    } else {
        __SetIn1PWM(htim, MAX_SPEED);
        __SetIn2PWM(htim,MAX_SPEED - speed);
    }
}

/**
 * @brief 控制电机后退
 * @param speed 速度值（0-100）
 */
void motor520_Backward(TIM_HandleTypeDef *htim, uint8_t speed)
{
    if (speed > MAX_SPEED)
        speed = MAX_SPEED;
    
    if (currentDecayMode == FAST_DECAY) {
        __SetIn1PWM(htim, 0);
        __SetIn2PWM(htim, speed);
    } else {
        __SetIn1PWM(htim, MAX_SPEED - speed);
        __SetIn2PWM(htim, MAX_SPEED);
    }
}

/**
 * @brief 电机刹车
 */
void motor520_Brake(TIM_HandleTypeDef *htim)
{
    __SetIn1PWM(htim, MAX_SPEED);
    __SetIn2PWM(htim, MAX_SPEED);
}

/**
 * @brief 电机滑行
 */
void motor520_Coast(TIM_HandleTypeDef *htim)
{
    __SetIn1PWM(htim, 0);
    __SetIn2PWM(htim, 0);
}

/**************************************************以下为闭环模式********************************************/

/**
 * @brief 获取当前编码器值
 */
void motor520_GET_Encoder(TIM_HandleTypeDef *htim, uint32_t *val)
{
	*val = __HAL_TIM_GET_COUNTER(htim);
}

