#include <stdbool.h>
#include "step.h"
/**
 * @brief 初始化/重置五次多项式规划器
 * @param seg 指向结构体的指针
 * @param p0, v0, a0 起始位置、速度、加速度
 * @param pT, vT, aT 目标位置、速度、加速度
 * @param duration 动作总时长 (秒)
 * @param current_tick 当前系统时间 (ms)
 */
void Quintic_SetTrajectory(QuinticParam_t *seg,
                           float p0, float v0, float a0,
                           float pT, float vT, float aT,
                           float duration, uint32_t current_tick)
{
    // 确保最小的时长为0.05秒
    if (duration < 0.05f) duration = 0.05f;
    
    // 限制目标速度在最大电机速度范围内
    if (vT > M3508_MAX_SPEED_RADS) vT = M3508_MAX_SPEED_RADS;
    if (vT < -M3508_MAX_SPEED_RADS) vT = -M3508_MAX_SPEED_RADS;
    
    float T = (float)duration;
    float T2 = T * T;
    float T3 = T2 * T;
    float T4 = T3 * T;
    float T5 = T4 * T;

    seg->T = duration;
    seg->start_pos = p0;
    seg->target_pos = pT;
    seg->start_time_ms = current_tick;
    seg->is_running = 1;

    // 计算五次多项式的系数
    seg->a = p0;
    seg->b = v0;
    seg->c = 0.5f * a0;
    seg->d = (10.0f * (pT - p0) - (6.0f * v0 + 4.0f * vT) * T - (1.5f * a0 - 0.5f * aT) * T2) / T3;
    seg->e = (-15.0f * (pT - p0) + (8.0f * v0 + 7.0f * vT) * T + (1.5f * a0 - aT) * T2) / T4;
    seg->f = (6.0f * (pT - p0) - (3.0f * v0 + 3.0f * vT) * T - (0.5f * a0 - 0.5f * aT) * T2) / T5;
}
/**
 * @brief 获取当前轨迹状态（位置、速度、加速度）
 * @param seg 指向结构体的指针
 * @param current_tick 当前系统时间 (ms)
 * @param out 指向输出状态结构体的指针
 */
void Quintic_GetFullState(QuinticParam_t* seg, uint32_t current_tick, TrajectoryState_t* out)
{
    if (!seg->is_running || (current_tick < seg->start_time_ms)) {
        out->pos = seg->target_pos;
        out->vel = 0.0f;
        out->acc = 0.0f;
        return;
    }
    float t = (float)(current_tick - seg->start_time_ms) * 0.001f;
    if (t >= seg->T) {
        seg->is_running = 0;
        out->pos = seg->target_pos;
        out->vel = 0.0f;
        out->acc = 0.0f;
        return;
    }
    out->pos = ((((seg->f * t + seg->e) * t + seg->d) * t + seg->c) * t + seg->b) * t + seg->a;
    out->vel = (((5.0f * seg->f * t + 4.0f * seg->e) * t + 3.0f * seg->d) * t + 2.0f * seg->c) * t + seg->b;
    out->acc = ((20.0f * seg->f * t + 12.0f * seg->e) * t + 6.0f * seg->d) * t + 2.0f * seg->c;
}
