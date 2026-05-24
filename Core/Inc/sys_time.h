/**
 * @file sys_time.h
 * @brief 系统时间接口 —— 封装 HAL_GetTick，DWT 微秒计数器
 */

#ifndef __SYS_TIME_H__
#define __SYS_TIME_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* nowtime removed from sys_time.h (v3.5+) — IMU uses DWT directly */

/**
 * @brief 获取系统上电后毫秒数
 */
uint32_t SysTime_GetMs(void);

/**
 * @brief 获取系统上电后微秒数
 */
uint32_t SysTime_GetUs(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYS_TIME_H__ */
