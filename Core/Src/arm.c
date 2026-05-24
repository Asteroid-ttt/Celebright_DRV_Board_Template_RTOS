/**
 ******************************************************************************
 * @file    arm.c
 * @brief   平面二杆机械臂相关计算的实现
 * @author  Zhanng
 * @version 1.0
 ******************************************************************************
 * Copyright (c) 2025.5 Team Inno-WHU
 * All rights reserved.
 *
 ******************************************************************************
 */

#include "app.h"

#if APP_ENABLE_ROBOTIC_ARM

#include "arm.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
// 定义常量
#define PI (3.14159265358979323846f)

#define EPSILON 1e-4f // 浮点比较允许的误差
// 定义舵机最大的角度 使用弧度制
#define MAX_ANGLE 5.0/6.0 * PI
#define MIN_ANGLE -5.0/6.0 * PI



uint8_t _initRoboticArm(RoboticArm* arm,
                        uint16_t H, uint16_t link1, uint16_t link2, uint16_t link3,
                        float joint1, float joint2, float joint3, float joint4
                        ,ArmThirdAngleType theta3_type) {

    if (arm == NULL || link1 <= 0 || link2 <= 0 || link3 <= 0) {
        return false;
    }

    arm->H              =       H;
    arm->link1             =       link1;
    arm->link2             =       link2;
    arm->link3             =       link3;
    arm->angle          =       joint4;
    arm->theta1         =       joint1;
    arm->theta2         =       joint2;
    arm->theta3         =       joint3;
    arm->theta3_type    =       theta3_type;
    arm->isGripping     =       false;
    arm->status         =       WAIT;
    return true;
}

uint8_t _setArmType(RoboticArm* arm, ArmThirdAngleType type) {
    int typeid = (int)type;
    if (arm != NULL && (0 <= typeid && typeid <= 4)) {
        arm->theta3_type = type;
        return true;
    }else return false;
}

uint8_t _isValidArmConfiguration(const RoboticArm* arm) {
    if (arm == NULL) {
        return false;
    }

    // 检查角度是否在有效范围内
    if (arm->theta1 < MIN_ANGLE || arm->theta1 > MAX_ANGLE ||
        arm->theta2 < MIN_ANGLE || arm->theta2 > MAX_ANGLE ||
        arm->theta3 < MIN_ANGLE || arm->theta3 > MAX_ANGLE ||
        arm->angle < MIN_ANGLE || arm->angle > MAX_ANGLE) {
        return false;
    }

    return true;
}
uint8_t _isValidArmConfiguration_Angle(const V_Angle * arm) {
    if (arm == NULL) {
        return false;
    }

    // 检查角度是否在有效范围内
    if (arm->theta1 < MIN_ANGLE || arm->theta1 > MAX_ANGLE ||
        arm->theta2 < MIN_ANGLE || arm->theta2 > MAX_ANGLE ||
        arm->theta3 < MIN_ANGLE || arm->theta3 > MAX_ANGLE ||
        arm->angle < MIN_ANGLE || arm->angle > MAX_ANGLE) {
        return false;
    }

    return true;
}


float _degreesToRadians(float degrees) {
    return degrees * PI / 180.0f;
}

float _radiansToDegrees(float radians) {
    return radians * 180.0f / PI;
}

float _getThirdArmAngle(ArmThirdAngleType type) {
    switch (type) {
        case U_VERTICAL:
            return PI / 2.0f;  // 90度 垂直向上
        case U_DEGREE_45:
            return PI / 4.0f;  // 45度 向上45度
        case HORIZONTAL:
            return 0.0f;       // 0度 水平
        case D_DEGREE_45:
            return -PI / 4.0f; // -45度 向下45度
        case D_VERTICAL:
            return -PI / 2.0f; // -90度 垂直向下
        default:
            return 0.0f;
    }
}

Vf_Position _forwardKinematics(const RoboticArm* arm, const V_Angle* angle) {
    Vf_Position pos = {0, 0, 0};

    if (arm == NULL) {
        return pos;
    }


    // 计算末端位置
    float x1 = arm->link1 * cosf(angle->theta1 + PI / 2);
    float z1 = arm->H + arm->link1 * sinf(angle->theta1 + PI / 2);

    // 奇变偶不变
    float x2 = x1 + arm->link2 * cosf(angle->theta1 + angle->theta2 + PI / 2);
    float z2 = z1 + arm->link2 * sinf(angle->theta1 + angle->theta2 + PI / 2);

    float x3 = x2 + arm->link3 * cosf(angle->theta1 + angle->theta2 + PI / 2 + angle->theta3);
    float z3 = z2 + arm->link3 * sinf(angle->theta1 + angle->theta2 + PI / 2 + angle->theta3);

    // 考虑底座旋转
    pos.x = x3 * sinf(angle->angle);
    pos.y = x3 * cosf(angle->angle);
    pos.z = z3;

    return pos;
}

// 待优化
uint8_t _inverseKinematics(RoboticArm* arm, const V_Position* target, V_Angle * angle) {

    if (arm == NULL || target == NULL || angle == NULL) {
        return false;
    }

    if(target->z < 0){
        angle->theta1 = arm->theta1;
        angle->theta2 = arm->theta2;
        angle->theta3 = arm->theta3;
        angle->angle = arm->angle;
        return false;
    }

    // 首先计算底座旋转角度
    float baseAngle = atan2f(target->x, target->y);

    // 计算在x-z平面（旋转后的平面）上的投影距离
    float projectedDist = sqrtf((target->x * target->x + target->y * target->y));

    // 考虑第三段臂的偏移
    float theta3 = _getThirdArmAngle(arm->theta3_type);
    float L3_x = arm->link3 * cosf(theta3);
    float L3_z = arm->link3 * sinf(theta3);

    // 计算第二段臂末端的目标位置
    float targetX = projectedDist - L3_x;
    float targetY = target->z - L3_z;

    // 调整目标高度，将底座高度考虑进去
    targetY -= arm->H;

    // 验证目标位置是否在工作范围内
    float distSq = targetX * targetX + targetY * targetY;
    float reach = arm->link1 + arm->link2;  // 最大伸展距离
    float minReach = fabs(arm->link1 - arm->link2);  // 最小伸展距离

    if (distSq > reach * reach || distSq < minReach * minReach - EPSILON) {
        // 目标点超出工作范围
        return false;
    }

    float dist = sqrtf(distSq);
    if (dist < EPSILON) {
        return false;  /* target too close to origin → no valid IK solution */
    }
    float Theta = atan2f(targetY, targetX);

    float arg1 = (targetX * targetX + targetY * targetY + arm->link1 * arm->link1 - arm->link2 * arm->link2) / (2.0f * arm->link1 * dist);
    float arg2 = (targetX * targetX + targetY * targetY + arm->link2 * arm->link2 - arm->link1 * arm->link1) / (2.0f * arm->link2 * dist);
    if (arg1 > 1.0f) arg1 = 1.0f;  if (arg1 < -1.0f) arg1 = -1.0f;
    if (arg2 > 1.0f) arg2 = 1.0f;  if (arg2 < -1.0f) arg2 = -1.0f;

    float tmpTheta1 = acosf(arg1);
    float tmpTheta2 = acosf(arg2);

    float Theta1 = Theta + tmpTheta1;
    float Theta2 = Theta - Theta1 - tmpTheta2;
    float Theta3 = theta3 - (Theta1 + Theta2 + PI / 2.0f); /* subtract PI/2 for the world-frame reference of link3 */

    angle->theta1 =  Theta1 - PI / 2;
    angle->theta2 = Theta2;
    angle->theta3 = Theta3;
    angle->angle = baseAngle;

    return true;
}


///////////////////////////////// 目前无用的函数
///////////////////////////////// 目前无用的函数
///////////////////////////////// 目前无用的函数
///////////////////////////////// 目前无用的函数
///////////////////////////////// 目前无用的函数
///////////////////////////////// 目前无用的函数
///////////////////////////////// 目前无用的函数
///////////////////////////////// 目前无用的函数
///////////////////////////////// 目前无用的函数
///////////////////////////////// 目前无用的函数
///////////////////////////////// 目前无用的函数
///////////////////////////////// 目前无用的函数
///////////////////////////////// 目前无用的函数
///////////////////////////////// 目前无用的函数


uint8_t
_setArm(RoboticArm* arm, uint16_t x, uint16_t y, uint16_t z) {
    if (arm == NULL) {
        return false;
    }
    V_Angle angle = {0,0,0,0};
    V_Position target = {x, y, z};
    uint8_t result = _inverseKinematics(arm, &target, &angle);
    if(result){
        _setArmAngle(arm, &angle);
        return true;
    }else{
        return false;
    }
}

void _printArmStatus(const RoboticArm* arm) {
    if (arm == NULL) {
        printf("机械臂状态：无效\n");
        return;
    }

    printf("机械臂当前状态：\n");
    printf("底座角度: %.2f度\n", _radiansToDegrees(arm->angle));
    printf("第一段臂角度: %.2f度\n", _radiansToDegrees(arm->theta1));
    printf("第二段臂角度: %.2f度\n", _radiansToDegrees(arm->theta2));
    printf("第三段臂角度: %.2f度\n", _radiansToDegrees(arm->theta3));

    float a1 = arm->theta1 + _degreesToRadians(90);
    float a2 = a1 + arm->theta2;
    float a3 = a2 + arm->theta3; // 如果程序正确 这里的a3应该会是一个特定的值
    printf("Local yaw angles (deg): %.2f %.2f %.2f\n",
           _radiansToDegrees(a1), _radiansToDegrees(a2), _radiansToDegrees(a3));

    // 打印第三段臂角度类型
    printf("Third arm angle type: ");
    switch (arm->theta3_type) {
        case U_VERTICAL:
            printf("Vertical Up (90 deg)\n");
            break;
        case U_DEGREE_45:
            printf("Up 45 deg\n");
            break;
        case HORIZONTAL:
            printf("Horizontal (0 deg)\n");
            break;
        case D_DEGREE_45:
            printf("Down 45 deg\n");
            break;
        case D_VERTICAL:
            printf("Vertical Down (-90 deg)\n");
            break;
        default:
            printf("未知\n");
    }

//    // 计算当前末端位置（正向运动学）
//    V_Position pos = _forwardKinematics(arm);
//    printf("末端位置: (%.2f, %.2f, %.2f) mm\n", pos.x, pos.y, pos.z);
}


uint8_t _setArmAngle(RoboticArm* arm, const V_Angle* angle) {
    if (arm == NULL || angle == NULL) {
        return false;
    }

    // 检查新目标角度是否在有效范围内
    if (!_isValidArmConfiguration_Angle(angle)) {
        return false;
    }

    arm->theta1 = angle->theta1;
    arm->theta2 = angle->theta2;
    arm->theta3 = angle->theta3;
    arm->angle = angle->angle;

    return true;
}

#endif /* APP_ENABLE_ROBOTIC_ARM */
