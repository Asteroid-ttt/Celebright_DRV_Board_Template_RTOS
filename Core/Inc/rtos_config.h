/**
 * @file rtos_config.h
 * @brief FreeRTOS 任务周期与模式开关
 */

#ifndef __RTOS_CONFIG_H__
#define __RTOS_CONFIG_H__

/* FreeRTOS 各任务执行间隔 (ms) */
#define TASK_ITV_CAR        10
#define TASK_ITV_IMU        5
#define TASK_ITV_UPLOAD     50

/* 启用小车控制（位置 PID 闭环） */
#define USE_CAR_CONTROL     1

/* 启用角速度 PID */
#define V_ANGLE_PID         1

/* IMU 作为角速度来源（否则使用编码器差速） */
#define V_DEGREE_FROM_IMU   1

#endif /* __RTOS_CONFIG_H__ */
