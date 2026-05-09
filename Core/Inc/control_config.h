/**
 * @file control_config.h
 * @brief 小车运动控制 PID 参数与限幅 —— 位置/角度/角速度环
 */

#ifndef __CONTROL_CONFIG_H__
#define __CONTROL_CONFIG_H__

#include "platform_config.h"

/* ====== 增量式 PID 限幅 ====== */
#define LIMIT_INC_POS           (MAX_V_REAL * 0.5F)
#define LIMIT_INC_SPIN          MAX_V_ANGLE
#define LIMIT_INC_V_ANGLE       (0.5F * MAX_V_ANGLE * FRAME_W_HALF * DEGREE_TO_RAD)
#define LIMIT_INC_VITUAL_X      0.0F
#define LIMIT_INC_VITUAL_Y      0.0F

/* ====== 位置式 PID 输出限幅 ====== */
#define LIMIT_POS_POS           MAX_V_REAL
#define LIMIT_POS_SPIN          MAX_V_ANGLE
#define LIMIT_POS_V_ANGLE       (MAX_V_ANGLE * FRAME_W_HALF * DEGREE_TO_RAD)
#define LIMIT_POS_VITUAL_X      100.0F
#define LIMIT_POS_VITUAL_Y      100.0F

/* ====== 位置式 PID 积分限幅 ====== */
#define LIMIT_ITGR_POS          LIMIT_ITGR_MAX
#define LIMIT_ITGR_SPIN         LIMIT_ITGR_MAX
#define LIMIT_ITGR_V_ANGLE      LIMIT_ITGR_MAX
#define LIMIT_ITGR_VITUAL_X     LIMIT_ITGR_MAX
#define LIMIT_ITGR_VITUAL_Y     LIMIT_ITGR_MAX

/* ====== 控制 PID 参数 (P/I/D) ====== */
#define P_POS           2.0F
#define P_SPIN          2.0F
#define P_V_ANGLE       1.0F

#define I_POS           0.01F
#define I_SPIN          0.01F
#define I_V_ANGLE       0.63F

#define D_POS           0.1F
#define D_SPIN          0.5F
#define D_V_ANGLE       0.01F

#endif /* __CONTROL_CONFIG_H__ */
