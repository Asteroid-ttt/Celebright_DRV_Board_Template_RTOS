/**
 ******************************************************************************
 * @file    arm.h/c
 * @brief   平面二杆机械臂相关计算
 * @author  Zhanng
 * @version 1.0
 ******************************************************************************
 * Copyright (c) 2025.5 Team Inno-WHU
 * All rights reserved.
 *
 ******************************************************************************
 */

#ifndef FOURAXISPLANARTWOLINKROBOTICARM_ARM_H
#define FOURAXISPLANARTWOLINKROBOTICARM_ARM_H

#include <stdint.h>

/**
 ******************************************************************************
 *
 * 相关说明
 * 机械为四自由度机械臂 使用平面二杆
 * 从机械臂的视角 坐标系如下;
 * |
 * |
 * |
 * |
 * @ 小车正前方--- > --------> Y
 * |
 * |
 * |
 * V
 * X
 *
 * 所有关节正向摆放的时候 正为向天空 负为向地面
 * 计算代码中所有的角度表示为弧度制 所有显示相关和使用相关的为角度制
 * 距离相关的参数全部毫米
 * 通过提前设定机械臂的最后一段 可以实现简化为一个平面二杆机械臂
 * 距离由目标位置减去最后一段决定
 ******************************************************************************
 */



/**
 ******************************************************************************
 * 数据结构
 ******************************************************************************
 */

// 定义第三段臂的固定角度类型
typedef enum {
    U_VERTICAL = 0,    // 垂直向上
    U_DEGREE_45 = 1,    // 向上45度
    HORIZONTAL = 2,  // 水平
    D_DEGREE_45 = 3,    // 向下45度
    D_VERTICAL = 4,    // 垂直向下
} ArmThirdAngleType;


typedef enum{
    WAIT = 0,
    A_LEFT = 1,
    A_RIGHT = 2,
} ArmStatus;

// 机械臂参数结构体
typedef struct {
    uint16_t H;                     // 底座高度
    uint16_t link1;                 // 第一段臂长度
    uint16_t link2;                 // 第二段臂长度
    uint16_t link3;                 // 第三段臂长度
    float angle;                    // 底座的旋转角度
    float theta1;                   // 第一段臂角度（弧度）
    float theta2;                   // 第二段臂角度（弧度）
    float theta3;                   // 第三段角度 (弧度)
    ArmThirdAngleType theta3_type;  // 第三段臂角度类型
    uint8_t isGripping;             // 表示当前夹子是否加紧
    uint8_t ObjWidth;               // 物体宽度
    ArmStatus status;               // 当前机械臂的状态
} RoboticArm;

// 末端执行器位置
typedef struct {
    uint16_t x;    // x坐标
    uint16_t y;    // y坐标
    uint16_t z;    // z坐标
} V_Position;

typedef  struct{
    float x;
    float y;
    float z;
}Vf_Position;

// 四个自由度的角度
typedef struct{
    float theta1;    // 第一段臂角度（弧度）
    float theta2;    // 第二段臂角度（弧度）
    float theta3;    // 第三段臂角度（弧度）
    float angle;     // 底座旋转角度（弧度）
} V_Angle;

/**
 ******************************************************************************
 * 接口函数
 ******************************************************************************
 */

/**
 * 机械臂移动到相关的位置
 * 调用流程：
 * 1. 创建目标位置
 * 2. 调用 _inverseKinematics() 计算所需角度
 * 3. 调用 _setArmAngle() 更新机械臂参数
 * 4. 调用 _moveArmAngle() 执行实际移动
 * @param arm 机械臂结构
 * @param xyz 位置信息 单位mm
 * @return 是否移动成功
 */

uint8_t
_setArm(RoboticArm* arm, uint16_t x, uint16_t y, uint16_t z);

/**
 * 设置机械臂末端的类型
 * @param arm 机械臂结构
 * @param type 类型
 * @return 设置是否成功
 */
uint8_t _setArmType(RoboticArm* arm, ArmThirdAngleType type);

/**
 ******************************************************************************
 * 设置相关辅助函数
 ******************************************************************************
 */

/**
 * 初始化机械臂结构
 * @param arm 机械臂结构
 * @param H 底座高度
 * @param L1 第一段臂长度
 * @param L2 第二段臂长度
 * @param L3 第三段臂长度
 * @param theta3_type 第三段臂角度类型
 * @return 初始化是否成功
 */
uint8_t _initRoboticArm(RoboticArm* arm,
                        uint16_t H, uint16_t link1, uint16_t link2, uint16_t link3,
                        float joint1, float joint2, float joint3, float joint4 ,ArmThirdAngleType theta3_type);

/**
 * 设置机械臂角度（弧度）
 *
 * @param arm 机械臂结构
 * @param angle 设置的角度
 * @return 是否设置成功
 */
uint8_t _setArmAngle(RoboticArm* arm, const V_Angle* angle);


/**
 * 检查机械臂配置是否有效
 *
 * @param arm 机械臂结构
 * @return 配置是否有效
 */
uint8_t _isValidArmConfiguration(const RoboticArm* arm);
uint8_t _isValidArmConfiguration_Angle(const V_Angle * arm);

/**
 * 获取第三段臂的角度（弧度）
 *
 * @param type  类型
 * @return 弧度制
 */
float _getThirdArmAngle(ArmThirdAngleType type);

/**
 ******************************************************************************
 * 数学相关
 ******************************************************************************
 */

/**
 * 角度转换：度到弧度
 *
 * @param degrees 角度值（度）
 * @return 角度值（弧度）
 */
float _degreesToRadians(float degrees);

/**
 * 角度转换：弧度到度
 *
 * @param radians 角度值（弧度）
 * @return 角度值（度）
 */
float _radiansToDegrees(float radians);


/**
 * 正向运动学：根据关节角度计算末端位置
 *
 * @param arm 机械臂结构
 * @return 末端执行器位置
 */
Vf_Position _forwardKinematics(const RoboticArm* arm, const V_Angle* angle);

/**
 * 逆向运动学：根据末端位置计算关节角度
 *
 * @param arm 机械臂结构
 * @param target 目标位置
 * @return 是否成功计算（如果目标点超出范围则返回false）
 */
uint8_t _inverseKinematics(RoboticArm* arm, const V_Position* target, V_Angle * angle);


/**
 *
 ******************************************************************************
 * 显示相关
 ******************************************************************************
 */

/**
 * 打印机械臂当前状态
 *
 * 调用流程：
 * 1. 转换并显示当前各关节角度（弧度转角度）
 * 2. 显示第三段臂角度类型
 * 3. 调用 _forwardKinematics() 计算并显示末端位置
 *
 * 包含机械臂当前的角度信息 (角度制)
 * 机械臂的末端的位置 (正向)
 * 机械臂的误差
 *
 * @param arm 机械臂结构
 */
void _printArmStatus(const RoboticArm* arm);


#endif //FOURAXISPLANARTWOLINKROBOTICARM_ARM_H
