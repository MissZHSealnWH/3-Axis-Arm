
#ifndef __PWMmotor_H_
#define __PWMmotor_H_

#include <stdint.h>
#include "tim.h"

/**
 * @brief 衰减模式枚举
 */
typedef enum {
    SLOW_DECAY,  /**< 慢衰减模式 */
    FAST_DECAY   /**< 快衰减模式 */
} DecayMode;

/**
 * @brief 初始化
 */
void motor520_Init(TIM_HandleTypeDef *htim);

/**
 * @brief 设置衰减模式
 * @param mode 衰减模式
 */
void motor520_SetDecayMode(DecayMode mode);

/**
 * @brief 控制电机前进
 * @param speed 速度值（0-100）
 */
void motor520_Forward(TIM_HandleTypeDef *htim, uint8_t speed);

/**
 * @brief 控制电机后退
 * @param speed 速度值（0-100）
 */
void motor520_Backward(TIM_HandleTypeDef *htim, uint8_t speed);

/**
 * @brief 电机刹车
 */
void motor520_Brake(TIM_HandleTypeDef *htim);

/**
 * @brief 电机滑行
 */
void motor520_Coast(TIM_HandleTypeDef *htim);

/**************************************************以下为闭环模式********************************************/

/**
 * @brief 获取当前编码器值
 */
void motor520_GET_Encoder(TIM_HandleTypeDef *htim, uint32_t *val);
	
#endif /* __DRV8833_H_ */
