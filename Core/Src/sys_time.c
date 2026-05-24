/**
 * @file sys_time.c
 * @brief 系统时间实现 —— 基于 HAL_GetTick() + DWT 周期计数器
 */

#include "sys_time.h"
#include "main.h"

/* nowtime removed — IMU timing now uses DWT->CYCCNT directly (v3.5+) */

static uint32_t cpu_freq_hz = 480000000;

uint32_t SysTime_GetMs(void)
{
    return HAL_GetTick();
}

uint32_t SysTime_GetUs(void)
{
    return DWT->CYCCNT / (cpu_freq_hz / 1000000);
}
