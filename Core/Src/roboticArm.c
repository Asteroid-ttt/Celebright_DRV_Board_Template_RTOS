/*
 * @file roboticArm.c
 * @brief 机械臂舵机硬件接口模块，包含机械臂初始化、舵机控制及安全限位功能
 * @note 本模块依赖于SCServo库实现底层舵机控制，机械臂各关节角度限位数据由外部常量数组angle_limits提供（定义在roboticArm.h）
 * @note 本模块为硬件接口模块，上层应用位于ARobot文件
 */


#include "app.h"
#include "roboticArm.h"
#if APP_ENABLE_ROBOTIC_ARM
#include "main.h"
#include "SCServo.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

const ArmAngleLimit angle_limits[5] = {
    {1,  ARM_ANGLE_LIMIT_1_MAX,  ARM_ANGLE_LIMIT_1_MIN, ARM_ANGLE_INIT_1_init_state},
    {2,  ARM_ANGLE_LIMIT_2_MAX,  ARM_ANGLE_LIMIT_2_MIN, ARM_ANGLE_INIT_2_middle_state},
    {88, ARM_ANGLE_LIMIT_88_MAX, ARM_ANGLE_LIMIT_88_MIN, ARM_ANGLE_INIT_88_middle_state},
    {9,  ARM_ANGLE_LIMIT_9_MAX,  ARM_ANGLE_LIMIT_9_MIN, ARM_ANGLE_INIT_9_middle_state},
    {66, ARM_ANGLE_LIMIT_66_MAX, ARM_ANGLE_LIMIT_66_MIN, ARM_ANGLE_INIT_66_init_state}
};


/**​
* @brief 机械臂上电位置初始化
* @note 考虑到小车平衡及机械结构，上电初始化是一个顺序不可打乱过程
 */
void arm_init(void){
	
 int init_Pos_1;//初始舵机1位置
 HAL_Delay(1000);
 init_Pos_1 = ReadPos(1); //读取舵机1位置反馈
 //确定机械臂相对小车左右
 if(init_Pos_1 <= ARM_ANGLE_INIT_1_init_state)//机械臂在车左侧时候
    {
        init_Pos_1 = ARM_ANGLE_INIT_1_left;
    }
    else                                   //机械臂在车右侧时候
    {
        init_Pos_1 = ARM_ANGLE_INIT_1_right;
    }
	

    // 依次初始化各个关节位置
	WritePos(1,init_Pos_1, 0, 100);
    HAL_Delay(6000);
    WritePos(2,433, 0, 100);
	HAL_Delay(6000);
    WritePos(88,ARM_ANGLE_INIT_88_fst, 0, 100);
    HAL_Delay(6000);
    WritePos(2,ARM_ANGLE_INIT_2_snd, 0, 100);
    HAL_Delay(6000);
    WritePos(1,ARM_ANGLE_INIT_1_init_state, 0, 100);
    HAL_Delay(6000);
    WritePos(88,ARM_ANGLE_INIT_88_init_state, 0, 100);
    HAL_Delay(6000);
    WritePos(2,ARM_ANGLE_INIT_2_init_state, 0, 100);
    HAL_Delay(6000);
    WritePos(9,ARM_ANGLE_INIT_9_init_state, 0, 100);
    HAL_Delay(6000);
}

void arm_init_forRTOS(void){
    int init_Pos_1;//初始舵机1位置
    osDelay(1000);
    init_Pos_1 = ReadPos(1); //读取舵机1位置反馈
    if(init_Pos_1 <= ARM_ANGLE_INIT_1_init_state)//机械臂在车左侧时候
        {
            init_Pos_1 = ARM_ANGLE_INIT_1_left;
        }
        else                                   //机械臂在车右侧时候
        {
            init_Pos_1 = ARM_ANGLE_INIT_1_right;
        }
	

    // 依次初始化各个关节位置
	WritePos(1,init_Pos_1, 0, 100);
    osDelay(6000);
    // 这里的osDelay是FreeRTOS的延时函数，替代HAL
    WritePos(2,433, 0, 100);
	osDelay(6000);
    WritePos(88,ARM_ANGLE_INIT_88_fst, 0, 100);
    osDelay(6000);
    WritePos(2,ARM_ANGLE_INIT_2_snd, 0, 100);
    osDelay(6000);
    WritePos(1,ARM_ANGLE_INIT_1_init_state, 0, 100);
    osDelay(6000);
    WritePos(88,ARM_ANGLE_INIT_88_init_state, 0, 100);
    osDelay(6000);
    WritePos(2,ARM_ANGLE_INIT_2_init_state, 0, 100);
    osDelay(6000);
    WritePos(9,ARM_ANGLE_INIT_9_init_state, 0, 100);
    osDelay(6000);
}

 /**​
 * @brief （机械臂）舵机初始化
 */
void setup_roboticArm(void) {
   setEnd(1);//SCSCL舵机为大端存储结构
}


/**​
 * @brief 设置舵机绝对角度
 * @param ID 舵机ID
 * @param angle 目标角度（单位：度，范围0~360）
 * @note 角度转换为12位编码（0~4096），直接控制舵机到指定位置
 */
void set_servo_angle_roboticArm(uint8_t ID, float angle) {
	
    // 角度转舵机位置编码（300°对应1024步）
    int position = (int)(angle / 300.0f * 1024.0f); 
	
    //舵机(ID)以最高速度V=10*0.059=0.59rpm，运行至position位置(有机械大小限制）
    WritePos(ID,limit_angle(ID,position), 0, 100);
}
/**​
 * @brief 设置舵机相对角度
 * @param ID 舵机ID
 * @param angle 目标角度（单位：度，范围0~300）
 * @note 角度转换为12位编码（0~1024），直接控制舵机到指定位置
 */
void set_servo_angle_offset_roboticArm(uint8_t ID, float angle) {

    // 计算偏移量（角度→编码）
    int offset = (int)(angle / 300.0f * 1024.0f); 
    // 读取当前位置（需确保舵机反馈正常）
    int pre_position = ReadPos(ID); 
	int target_position = pre_position; /* fallback: stay at current position if ID unknown */
    // 计算目标位置并限制范围
	for (size_t i = 0; i < sizeof(angle_limits)/sizeof(angle_limits[0]); ++i)
	{
        if (angle_limits[i].id == ID) {
     target_position = angle_limits[i].middle + offset;
 			break;
		}
	}
    
    // 平滑移动至新位置（参数同绝对角度函数）
    WritePos(ID, limit_angle(ID,target_position), 0, 100);
}








/**​
 * @brief 限制关节位置
 * @param ID 舵机ID
 * @param position 目标位置
 * @note 不同关节不同限制，仅为机械结构限制
 */
int limit_angle(uint8_t ID,int position){
	
for (size_t i = 0; i < sizeof(angle_limits)/sizeof(angle_limits[0]); ++i)
	{
        if (angle_limits[i].id == ID) {
            // 获取当前关节的物理限位
             int max_limit = angle_limits[i].max;
             int min_limit = angle_limits[i].min;
            
            // 实施安全钳位（自动处理超过min/max的情况）
            position = (position > max_limit) ? max_limit : position;
            position = (position < min_limit) ? min_limit : position;
            break;
        }
	}
	return position;
}



#endif /* APP_ENABLE_ROBOTIC_ARM */
