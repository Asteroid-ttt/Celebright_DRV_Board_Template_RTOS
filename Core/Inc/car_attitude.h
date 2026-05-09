/**
 * @file car_attitude.h
 * @author cbr (ran13245@outlook.com)
 * @brief 小车控制代码
 * @version 0.0
 * @date 2023-06-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */

 #ifndef __CAR_ATTITUDE_H_
 #define __CAR_ATTITUDE_H_

 #include "pid.h"


//左右轮距一半的倒数    单位mm^-1
#define FRAME_W_HALF_REC (1.0F/FRAME_W_HALF)


 typedef struct __car_attitude
{
    float current_v_line;//当前线速度,mm/s
    float current_v_angle;//当前角速度,degree/s
    // float current_v_z;//旋转补偿线速度,左右差速值的一半mm/s
    float target_v_line;//目标线速度,mm/s
    float target_v_angle;//目标角速度,degree/s
    // float target_v_z;//旋转补偿线速度,mm/s
    _Bool flag_stop;//小车是否停止
    _Bool updated;//小车姿态是否更新
    pid pid_v_angle;
}_car_attitude;


extern _car_attitude car_attitude;//全局变量实例化

void Car_Attitude_Update_Output(void);
void Car_Attitude_Update_Input(void);
void Set_Car_Attitude(float v_line_target,float v_angle_target);
void init_Car_Attitude(void);

void Set_Car_Stop(void);
void Set_Car_Start(void);

void Car_Attitude_Yaw_Update(float v_angle,float time);
#endif


