#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "pid.h"


/***************待更改************************************/
//FreeRTOS更新速度任务的   时间间隔(s),时间间隔的倒数(s),频率
#define RTOS_MOTOR_TIME_ITV (TASK_ITV_CAR*0.001F)
#define RTOS_MOTOR_TIME_ITV_REC (1.0F/RTOS_MOTOR_TIME_ITV)
/***************待更改*************************************/

#define V_ENC_TO_REAL (1.0F/V_REAL_TO_ENC) //编码器速度转实际速度

typedef struct __motor
{   
    /*电机控制引脚相关*/
    GPIO_TypeDef* motor_gpio1;  // 存储 GPIO 端口（如 GPIOA）
    uint16_t      motor_pin1;   // 存储 GPIO 引脚（如 GPIO_PIN_8）
    GPIO_TypeDef* motor_gpio2;  
    uint16_t      motor_pin2;   

    /*PWM输出相关*/
	TIM_HandleTypeDef *htim;//PWM输出定时器
	uint32_t Channel;//PWM输出定时器频道
	uint32_t dutyCycle;//PWM占空比（绝对值）
    
    /*电机方向标志位*/
    _Bool dir; // 电机方向，1为正转，0为反转   
   
    /*PID实例化*/
    pid v_pid;

    /*PID控制相关*/
    int EncSource;//中断读取的编码器变化数
    int total_enc;//累计编码器数
    float v_enc;//编码器速度
    float v_real;//实际速度
    float target_v_enc;//目标编码器速度
    float duty;//有符号占空比
}motor;


/*准确轮相关*/
typedef enum __motor_choice{
    LEFT=0,
    RIGHT
}_motor_choice;

extern motor motor_LeftFront;
extern motor motor_LeftRear;
extern motor motor_RightFront;
extern motor motor_RightRear;

// 函数声明
//void Motor_Init(void);   //初始化

/*电机控制辅助函数*/
void Motor_Forward(motor* motor);//正/倒/停函数
void Motor_Backward(motor* motor);
void Motor_Stop(motor* motor);
void Motor_SetPWM(motor* motor); //设置占空比

/*辅助函数*/
float myabs(float num);//求绝对值（占空比）
float duty_limit(float duty);//占空比限制


/*对外接口*/
void init_motor(void);
void Motor_Update_Input_All(void);
void Motor_Update_Output_All(void);
void Motor_Set_V_Real_All(float v_real_LF,float v_real_LR,float v_real_RF,float v_real_RR);
float Wheel_Get_V_Real(_motor_choice choice);
void Wheel_Set_V_Real(float v_real_left, float v_real_right);
float Wheel_Get_Distance(_motor_choice choice);
void Wheel_Clear_Distance(void);
void Motor_Set_V_Real(motor* motor,float v_real);
void Motor_Set_V_Enc(motor* motor,float v_enc);


#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
