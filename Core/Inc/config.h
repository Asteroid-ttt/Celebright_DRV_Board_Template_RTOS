
/**
 * @file config.h
 * @brief 系统配置聚合器 —— 保持向后兼容，统一引用所有域配置文件
 *
 * 推荐新代码按域分别 include：
 *   #include "platform_config.h"   (轮径/轮距/编码器/最大速度)
 *   #include "motor_config.h"      (4电机PID参数与限幅)
 *   #include "control_config.h"    (位置/角度/角速度PID参数与限幅)
 *   #include "rtos_config.h"       (FreeRTOS任务周期与模式开关)
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "platform_config.h"
#include "motor_config.h"
#include "control_config.h"
#include "rtos_config.h"

#endif /* __CONFIG_H__ */


