/**
 * @file sys_time.h
 * @brief 系统时间接口 —— 封装 HAL_GetTick，解耦全局 nowtime
 */

#ifndef __SYS_TIME_H__
#define __SYS_TIME_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 系统时间（100us 精度），由 sys_time.c 定义
 * @note  原定义于 main.c，现迁移至此以消除模块耦合
 */
extern uint32_t nowtime;

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
