#ifndef STEP_1D_H
#define STEP_1D_H

#include <stdbool.h>
#include <stdint.h>
#define M3508_MAX_SPEED_RADS 50.47f // 482 rpm 转换为 rad/s
#define M3508_MAX_ACCEL_RADS2 500.0f // 根据堵转扭矩估算的经验加速度上限
/**
 * @brief 五次多项式参数结构体
 */
typedef struct {
    float a, b, c, d, e, f; // 多项式系数
    float T;                // 规划总时长 (s)
    float start_pos;        // 起始位置
    float target_pos;       // 目标位置
    uint32_t start_time_ms;  // 记录开始时的系统滴答值 (ms)
    uint8_t is_running;      // 规划运行标志位
} QuinticParam_t;

typedef struct {
    float pos;
    float vel;
    float acc;
} TrajectoryState_t;
/**
 * @brief 初始化/重置五次多项式规划器
 * @param seg 指向结构体的指针
 * @param p0, v0, a0 起始位置、速度、加速度
 * @param pT, vT, aT 目标位置、速度、加速度
 * @param duration 动作总时长 (秒)
 * @param current_tick 当前系统时间 (ms)
 */
void Quintic_SetTrajectory(QuinticParam_t* seg, 
                           float p0, float v0, float a0, 
                           float pT, float vT, float aT, 
                           float duration, uint32_t current_tick);
/**
 * @brief 获取当前轨迹状态（位置、速度、加速度）
 * @param seg 指向结构体的指针
 * @param current_tick 当前系统时间 (ms)
 * @param out 指向输出状态结构体的指针
 */ 
void Quintic_GetFullState(QuinticParam_t* seg, uint32_t current_tick, TrajectoryState_t* out);
#endif
