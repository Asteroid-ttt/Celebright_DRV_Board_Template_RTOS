/**
 * @file platform_config.h
 * @brief 小车平台参数 —— 轮径、轮距、编码器线数、最大速度
 */

#ifndef __PLATFORM_CONFIG_H__
#define __PLATFORM_CONFIG_H__

/* 数学常量 */
#define RAD_TO_DEGREE   57.29578018F
#define DEGREE_TO_RAD   0.01745329238F

/* 电机个数：1=四轮, 0=两轮(前轮) */
#define USE_4_MOTOR             1

/* 编码器四倍频 */
#define USE_4_TIMES_ENCODER     1

/* 每圈编码器脉冲数 */
#define ENC_EVERY_CIRCLE        1061.268F

/* 轮径 mm */
#define WHEEL_DIR               47F

/* 轮周长 mm */
#define WHEEL_PERIMETER         148.722996F

/* 左右轮距的一半 mm（用 IMU 测算） */
#define FRAME_W_HALF            66.25F

/* 前后轮轴距的一半 mm */
#define FRAME_L_HALF            55.85F

/* 编码器速度 → 实际速度 换算系数 */
#define V_REAL_TO_ENC           (ENC_EVERY_CIRCLE / WHEEL_PERIMETER)

/* 电机目标编码器速度最大值 (enc/s) */
#define MAX_V_ENC               (2 * ENC_EVERY_CIRCLE)

/* 电机最大实际速度 (mm/s) */
#define MAX_V_REAL              (MAX_V_ENC / V_REAL_TO_ENC)

/* 最大角速度 (degree/s) */
#define MAX_V_ANGLE             60.0F

#endif /* __PLATFORM_CONFIG_H__ */
