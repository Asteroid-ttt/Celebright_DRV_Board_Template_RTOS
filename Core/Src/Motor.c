#include "motor.h"
#include "config.h"
#include "spi.h"//LCD屏幕显示
#include "PWM.h"


//实例化四个轮子
motor motor_LeftFront={0};
motor motor_LeftRear={0};
motor motor_RightFront={0};
motor motor_RightRear={0};
extern uint16_t dutyA , dutyB , dutyC , dutyD ;

/*实例化准确轮,只能用来读取,不能用来写入*/
motor* wheel_left;
motor* wheel_right;


 
//static void Motor_Set_V_Enc(motor* motor,float v_enc);
static void Motor_Update_Output(motor* motor);
static void Motor_Update_Input(motor* motor);
static float Motor_Get_V_Real(motor* motor);
static void Motor_Clear_Distance(motor* motor);
static float Motor_Get_Distance(motor* motor);

//static void Motor_Set_V_Real_All(float v_real_LF,float v_real_LR,float v_real_RF,float v_real_RR);
//static void Motor_Set_V_Enc_All(float v_enc_LF,float v_enc_LR,float v_enc_RF,float v_enc_RR);
static void Motor_Judge_Accuracy(void);

/*  
    函数目录
    1.核心功能实现
    2.批量设置函数
    3.对外接口函数
    4.辅助函数     
*/



/* 核心功能实现 ---------------------------------------------*/

/**
 * @brief 初始化所有电机参数
 * @note 包含方向、PID参数、GPIO引脚映射、PWM通道配置
 * @warning 还需要后续修改引脚！！！
 * @warning 还需要后续修改PID参数！！！
 */
void init_motor(void){
/*配置控制引脚*/
	// ain
motor_LeftFront.motor_gpio1 = GPIOE;
motor_LeftFront.motor_gpio2 = GPIOC;
motor_LeftFront.motor_pin1 = GPIO_PIN_3;
motor_LeftFront.motor_pin2 = GPIO_PIN_13;

	// bin
motor_LeftRear.motor_gpio2 = GPIOC;
motor_LeftRear.motor_gpio1 = GPIOC;
motor_LeftRear.motor_pin2 = GPIO_PIN_0;
motor_LeftRear.motor_pin1 = GPIO_PIN_3;

// din
motor_RightFront.motor_gpio1 = GPIOB;
motor_RightFront.motor_gpio2 = GPIOE;
motor_RightFront.motor_pin1 = GPIO_PIN_1;
motor_RightFront.motor_pin2 = GPIO_PIN_7;
	
// cin
motor_RightRear.motor_gpio2 = GPIOC;
motor_RightRear.motor_gpio1 = GPIOC;
motor_RightRear.motor_pin2 = GPIO_PIN_4;
motor_RightRear.motor_pin1 = GPIO_PIN_5;
///*配置PWM输出定时器及频道*/
//motor_LeftFront.duty = dutyA;
motor_LeftFront.Channel = 0;

//motor_LeftRear.duty = dutyB;
motor_LeftRear.Channel = 1;

//motor_RightFront.duty = dutyD;
motor_RightFront.Channel = 3;

//motor_RightRear.duty = dutyC;
motor_RightRear.Channel = 2;
/*配置方向*/
motor_LeftFront.dir = 1; // 正转
motor_LeftRear.dir = 1;  // 正转
motor_RightFront.dir = 1; // 正转
motor_RightRear.dir = 1;  // 正转

/*配置PID限幅，后续调整*/
Set_PID_Limit(&motor_LeftFront.v_pid, LIMIT_INC_LF, LIMIT_POS_LF, LIMIT_ITGR_LF);
Set_PID_Limit(&motor_LeftRear.v_pid, LIMIT_INC_LR, LIMIT_POS_LR, LIMIT_ITGR_LR);
Set_PID_Limit(&motor_RightFront.v_pid, LIMIT_INC_RF, LIMIT_POS_RF, LIMIT_ITGR_RF);
Set_PID_Limit(&motor_RightRear.v_pid, LIMIT_INC_RR, LIMIT_POS_RR, LIMIT_ITGR_RR);
/*配置PID参数，后续调整*/
Set_PID(&motor_LeftFront.v_pid, P_LF, I_LF, D_LF);
Set_PID(&motor_LeftRear.v_pid, P_LR, I_LR, D_LR);
Set_PID(&motor_RightFront.v_pid, P_RF, I_RF, D_RF);
Set_PID(&motor_RightRear.v_pid, P_RR, I_RR, D_RR);
}

/*!
 * @brief 设置电机目标真实速度,单位mm/s
 * @param motor 电机obj
 * @param v_real 目标速度mm/s,float
 */
void Motor_Set_V_Real(motor* motor,float v_real){
    int v_enc=V_REAL_TO_ENC*v_real;
    Motor_Set_V_Enc(motor,v_enc);
}

/*!
 * @brief 获取电机当前真实速度,单位mm/s
 * @param motor 电机obj
 * @return 当前速度mm/s,float
 */
float Motor_Get_V_Real(motor* motor){
    return motor->v_real;
}

/*!
 * @brief 设置电机目标编码器速度,单位 编码器单位/s
 * @param motor 电机obj
 * @param v_enc 编码器速度,float
 */
void Motor_Set_V_Enc(motor* motor,float v_enc){
    v_enc = v_enc > MAX_V_ENC ? MAX_V_ENC : v_enc;
    v_enc = v_enc < -MAX_V_ENC? -MAX_V_ENC: v_enc;
    motor->target_v_enc=v_enc;
}

/*!
 * @brief 更新电机输出,调用增量式pid
 * @param motor 电机obj
 * @warning 逻辑是PID传入速度和目标速度都经过绝对值处理，方向通过目标速度方向控制
 */
float temp = 0;
void Motor_Update_Output(motor* motor){
    // if(car_attitude.flag_stop){
    //     motor->duty=0;
    //     GPIOPinWrite(motor->DirBase1,motor->DirPin1,~(motor->DirPin1));
    //     GPIOPinWrite(motor->DirBase2,motor->DirPin2,~(motor->DirPin2));
    // }
    // else{
			 temp = PID_Cal_Inc(&(motor->v_pid),myabs(motor->v_enc),myabs(motor->target_v_enc));	
       motor->duty=temp;
       

        motor->dutyCycle = myabs(motor->duty);
        Motor_SetPWM(motor);

        if(motor->dir){
            if(motor->target_v_enc > 0){
                Motor_Forward(motor);
            }
            else {
                Motor_Backward(motor);
                
            }
        }
        else{
            if(motor->target_v_enc  > 0){
                Motor_Backward(motor);
            }
            else {
                
                Motor_Forward(motor);
                
            }
        }
    // }
    
}

/*!
 * @brief 更新电机当前速度
 * @param motor 电机obj
 */
void Motor_Update_Input(motor* motor){
    if(!motor->dir){
        motor->EncSource=-motor->EncSource;
    }
		
   // motor->v_enc=motor->EncSource*RTOS_MOTOR_TIME_ITV_REC;
    motor->v_enc = motor->EncSource/(TASK_ITV_CAR*0.001F); // 10ms内的脉冲数变化转换为每秒的速度
    motor->v_real=motor->v_enc*V_ENC_TO_REAL;//记录实际速度，用于显示
    motor->total_enc+=motor->EncSource;

}

/*!
 * @brief 清除电机累计位置
 * @param motor 
 */
void Motor_Clear_Distance(motor* motor){
    motor->total_enc=0;
}

/*!
 * @brief 获得电机累计位置,单位mm
 * @param motor 
 */
float Motor_Get_Distance(motor* motor){

    //return motor->total_enc*1.0F/ENC_EVERY_CIRCLE*WHEEL_PERIMETER;//返回总距离
	return motor->total_enc*1.0F/V_REAL_TO_ENC;
}

/*!
 * @brief 决定准确轮
 * @param  
 */
void Motor_Judge_Accuracy(void){
    #if USE_4_MOTOR
    /*左*/
        float target=wheel_left->target_v_enc;//前后轮的目标速度是一样的,选其一
        if(myabs(motor_LeftFront.v_enc - target) > myabs(motor_LeftRear.v_enc - target))
            wheel_left=&motor_LeftRear;
        else 
            wheel_left=&motor_LeftFront;
    /*右*/
        target=wheel_right->target_v_enc;
        if(myabs(motor_RightFront.v_enc - target) > myabs(motor_RightRear.v_enc - target))
            wheel_right=&motor_RightRear;
        else 
            wheel_right=&motor_RightFront;
    #endif
    }



/* 批量设置函数 -------------------------------------------------*/
/*!
 * @brief 更新所有电机速度
 * @param  
 */
void Motor_Update_Input_All(void){
    #if     USE_4_MOTOR    
        Motor_Update_Input(&motor_LeftRear);
        Motor_Update_Input(&motor_RightRear);
    #endif
        Motor_Update_Input(&motor_LeftFront);
        Motor_Update_Input(&motor_RightFront);
    
        Motor_Judge_Accuracy();
    }
    
    /*!
     * @brief 更新所有电机输出
     * @param  
     */
    void Motor_Update_Output_All(void){
    #if     USE_4_MOTOR
        Motor_Update_Output(&motor_LeftRear);
        Motor_Update_Output(&motor_RightRear);
    #endif
        Motor_Update_Output(&motor_LeftFront);
        Motor_Update_Output(&motor_RightFront);
    }
    
    /*!
     * @brief 批量设置电机真实速度,单位mm/s
     * @param v_real_LF 
     * @param v_real_LR 
     * @param v_real_RF 
     * @param v_real_RR 
     */
    void Motor_Set_V_Real_All(float v_real_LF,float v_real_LR,float v_real_RF,float v_real_RR){
    #if     USE_4_MOTOR
        Motor_Set_V_Real(&motor_LeftRear,v_real_LR);
        Motor_Set_V_Real(&motor_RightRear,v_real_RR);
    #endif
        Motor_Set_V_Real(&motor_LeftFront,v_real_LF);
        Motor_Set_V_Real(&motor_RightFront,v_real_RF);
    }
    
    /*!
     * @brief 批量设置电机编码器速度,一般不用
     * @param v_enc_LF 
     * @param v_enc_LR 
     * @param v_enc_RF 
     * @param v_enc_RR 
     */
    void Motor_Set_V_Enc_All(float v_enc_LF,float v_enc_LR,float v_enc_RF,float v_enc_RR){
    #if     USE_4_MOTOR
        Motor_Set_V_Enc(&motor_LeftRear,v_enc_LR);
        Motor_Set_V_Enc(&motor_RightRear,v_enc_RR);
    #endif
        Motor_Set_V_Enc(&motor_LeftFront,v_enc_LF);
        Motor_Set_V_Enc(&motor_RightFront,v_enc_RF);
    }
    
 /* 对外接口函数 -------------------------------------------------*/
    /*!
     * @brief 获取左侧或右侧的速度,mm/s
     * @param choice LEFT或RIGHT
     * @return 速度,float
     */
    float Wheel_Get_V_Real(_motor_choice choice){
        if (choice == LEFT){
            return Motor_Get_V_Real(wheel_left);
        }
        else{
            return Motor_Get_V_Real(wheel_right);
        }
    }
    
    /*!
     * @brief 设置左右两侧的速度,mm/s
     * @param v_real_left 
     * @param v_real_right 
     */
    void Wheel_Set_V_Real(float v_real_left, float v_real_right){
        Motor_Set_V_Real_All(v_real_left,v_real_left,v_real_right,v_real_right);
    }
    
    /*!
     * @brief 获取左侧或右侧的总距离,mm
     * @param choice LEFT或RIGHT
     * @return 距离,float
     */
    float Wheel_Get_Distance(_motor_choice choice){
        if (choice == LEFT){
            return Motor_Get_Distance(wheel_left);
        }
        else {
            return Motor_Get_Distance(wheel_right);
        }
    }
    
    /*!
     * @brief 清除距离累计
     * @param  
     */
    void Wheel_Clear_Distance(void){
        Motor_Clear_Distance(&motor_LeftFront);
        Motor_Clear_Distance(&motor_LeftRear);
        Motor_Clear_Distance(&motor_RightFront);
        Motor_Clear_Distance(&motor_RightRear);
    }
    


/* 辅助函数 -------------------------------------------------*/

/**
 * @brief 绝对值计算
 * @param num 输入值
 * @return float 绝对值
 */
 float myabs(float num){
    return num > 0 ? num : -num;  
}

/**
 * @brief PWM占空比限幅
 * @param duty 原始占空比
 * @return float 限制后的占空比

 */
 float duty_limit(float duty){
    float rtn=duty;
    rtn= rtn > MAX_MOTOR_DUTY ? MAX_MOTOR_DUTY : rtn;  // 上限限幅
    rtn= rtn <(-MAX_MOTOR_DUTY) ?(-MAX_MOTOR_DUTY) : rtn;  // 下限限幅
    return rtn;
}


/* 电机控制辅助函数 -------------------------------------------------*/
/**
 * 
 * @brief 下面包含控制电机的函数，包括正转、反转、停止和设置PWM输出。
 * 
 * 函数:
 * - Motor_Forward(motor* motor): 启动电机，正转。
 * - Motor_Backward(motor* motor): 启动电机，反转。
 * - Motor_Stop(motor* motor): 停止电机。
 * - Motor_SetPWM(motor* motor): 调节电机的PWM输出。
 * 
 * 电机控制通过使用HAL库函数设置GPIO引脚和PWM信号来实现。
 */
void Motor_Forward(motor* motor) // 电机启动，正转
{
	 
	HAL_GPIO_WritePin(motor->motor_gpio1,motor->motor_pin1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(motor->motor_gpio2,motor->motor_pin2, GPIO_PIN_RESET);
		
}

void Motor_Backward(motor* motor) // 电机启动，反转
{
	HAL_GPIO_WritePin(motor->motor_gpio1,motor->motor_pin1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->motor_gpio2,motor->motor_pin2, GPIO_PIN_SET);
   
}

void Motor_Stop(motor* motor) // 电机停
{
   HAL_GPIO_WritePin(motor->motor_gpio1,motor->motor_pin1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->motor_gpio2,motor->motor_pin2, GPIO_PIN_RESET);
}

void Motor_SetPWM(motor* motor)//调节PWM输出函数
{

    // 启动PWM信号
    // HAL_TIM_PWM_Start(motor->htim, motor->Channel);
    
    // 设置PWM占空比
    Set_PWM_Duty((uint8_t)motor->Channel, motor->duty);
}



