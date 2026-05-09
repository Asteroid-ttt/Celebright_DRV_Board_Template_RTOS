

/*
 * @file roboticArm.h
 * @brief 机械臂舵机参数配置文件
 * @note 该文件定义机械臂各关节的物理限位、初始化位置及运动参数
 *       修改限位参数需确保符合机械结构安全要求
 */


#ifndef ARM_ANGLE_LIMITS_H
#define ARM_ANGLE_LIMITS_H

#include "SCServo.h"
#define ARM_ANGLE_LIMIT_1_MAX 980
#define ARM_ANGLE_LIMIT_1_MIN 96
#define ARM_ANGLE_INIT_1_left 160//机械臂在车左侧时候
#define ARM_ANGLE_INIT_1_right 970//机械臂在车右侧时候
#define ARM_ANGLE_INIT_1_init_state 566//1的中位等于初始化位置

#define ARM_ANGLE_LIMIT_2_MAX 900
#define ARM_ANGLE_LIMIT_2_MIN 157
#define ARM_ANGLE_INIT_2_fst 433
#define ARM_ANGLE_INIT_2_snd 538
#define ARM_ANGLE_INIT_2_init_state 387
#define ARM_ANGLE_INIT_2_middle_state  525

#define ARM_ANGLE_LIMIT_88_MAX 911
#define ARM_ANGLE_LIMIT_88_MIN 171
#define ARM_ANGLE_INIT_88_fst 535
#define ARM_ANGLE_INIT_88_init_state 873
#define ARM_ANGLE_INIT_88_middle_state  535

#define ARM_ANGLE_LIMIT_9_MAX  928
#define ARM_ANGLE_LIMIT_9_MIN  194
#define ARM_ANGLE_INIT_9_init_state 300
#define ARM_ANGLE_INIT_9_middle_state  578

#define ARM_ANGLE_LIMIT_66_MAX  812
#define ARM_ANGLE_LIMIT_66_MIN  308
#define ARM_ANGLE_INIT_66_init_state 484

typedef struct {
    int id;
    int max;
    int min;
	int middle;//舵机中位，方便计算（中位时为直立状态）
} ArmAngleLimit;

/*​
 * @brief 机械臂全关节限位参数表
 * @note 该表必须与关节参数定义严格对应，修改时需同步更新相关宏定义
 */
extern const ArmAngleLimit angle_limits[5];

/* 机械臂控制函数声明 */
void setup_roboticArm(void);
void set_servo_angle_roboticArm(uint8_t ID, float angle);
void set_servo_angle_offset_roboticArm(uint8_t ID, float angle);
int limit_angle(uint8_t ID,int position);
void arm_init(void);
void arm_init_forRTOS(void);

#endif 




