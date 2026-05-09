/**
 * @file app_control.c
 * @brief 小车控制服务 —— 10ms 周期控制闭环
 *
 * 调用链:
 *   编码器读取 → Motor_Update_Input_All → Car_Attitude_Update_Input
 *   → Car_Control_Update_Input → Car_Control_Update_Output
 *   → Car_Attitude_Update_Output → Motor_Update_Output_All
 */
#include "app.h"

void AppCarControl_Task(void *argument)
{
    (void)argument;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = TASK_ITV_CAR;

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        /* 编码器速度更新 */
        Tim_Update_Enc_Speed();

        /* 电机输入更新（编码器 → 速度） */
        Motor_Update_Input_All();

        /* 小车姿态更新（线速度 + 角速度） */
        Car_Attitude_Update_Input();

        /* 小车控制状态更新（位置/角度进度） */
        Car_Control_Update_Input();

        /* 小车控制输出（位置 PID → 目标姿态） */
        Car_Control_Update_Output();

        /* 小车姿态输出（角速度 PID → 左右轮速度） */
        Car_Attitude_Update_Output();

        /* 电机输出更新（速度 PID → PWM 占空比） */
        Motor_Update_Output_All();
    }
}
