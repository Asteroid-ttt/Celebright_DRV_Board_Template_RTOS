//
// Created by zhanng on 2025/5/19.
//
#include "stm32h7xx_hal.h"  // 根据实际芯片型号选择，如 f1/f4/f7 等
#include "app.h"
#include <stdio.h>
#include <math.h>
#include "ARobot.h"
#if APP_ENABLE_ROBOTIC_ARM
#include "arm.h"
#include "roboticArm.h"
#include "cmsis_os.h"
RoboticArm Arm;

#define         BASE            220
#define         ARM1            132
#define         ARM2            215
#define         ARM3            130

#define         BASE_ID         1
#define         ARM1_ID         2
#define         ARM2_ID         88
#define         ARM3_ID         9
#define         G_ID            66

/*
 * 1 is 0.0 and 0.0
 * 2 is -40.4296875 and -0.705631162427395
 * 88 is 99.0234375 and 1.7282850210178224
 * 9 is -81.4453125 and -1.4214888634406941
 * */

#define         INIT_BASE       (0.0f)
#define         INIT_T1         (-0.705631162427395f)
#define         INIT_T2         (1.7282850210178224f)
#define         INIT_T3         (-1.4214888634406941f)


#define bool uint8_t


#define bias(x) (x - 90f)

#define DEBUG 0

#define debug(code) if(DEBUG) {code};



/////////////////////////////////
// 硬件初始化函数


void move(int id, float angle){
	
   set_servo_angle_offset_roboticArm(id,angle);
}
/////////////////////////////////



void initArm(void){
    _initRoboticArm(&Arm,
                    BASE, ARM1, ARM2, ARM3 ,
                    INIT_BASE, INIT_T1, INIT_T2, INIT_T3,
                    D_VERTICAL);
    arm_init();
}

void initArmForRTOS(void){
    _initRoboticArm(&Arm,
                    BASE, ARM1, ARM2, ARM3 ,
                    INIT_BASE, INIT_T1, INIT_T2, INIT_T3,
                    D_VERTICAL);
    arm_init_forRTOS();
}

/**
 * 调用舵机的接口实现机械臂移动
 *
 * @param arm 机械臂
 * @return void
 */
void moveArmAngle(RoboticArm* arm, V_Angle* angle){

    // 先动底座和ARM3
    move(BASE_ID, _radiansToDegrees(angle->angle));

    move(ARM3_ID, -90.0f);
	  HAL_Delay(6000);

    move(ARM1_ID, _radiansToDegrees(angle->theta1));

    move(ARM2_ID, _radiansToDegrees(angle->theta2));
	  HAL_Delay(6000);

    move(ARM3_ID, _radiansToDegrees(angle->theta3) + 90.0f);
	  HAL_Delay(6000);

    arm->angle = angle->angle;
    arm->theta1 = angle->theta1;
    arm->theta2 = angle->theta2;
    arm->theta3 = angle->theta3;
    if(angle->theta1 > 0) arm->status = A_LEFT;
    else arm->status = A_RIGHT;
}

void moveArmAngleForRTOS(RoboticArm* arm, V_Angle* angle){

    // 先动底座和ARM3
    move(BASE_ID, _radiansToDegrees(angle->angle));

    move(ARM3_ID, -90.0f);
	  osDelay(6000);

    move(ARM1_ID, _radiansToDegrees(angle->theta1));

    move(ARM2_ID, _radiansToDegrees(angle->theta2));
	  osDelay(6000);

    move(ARM3_ID, _radiansToDegrees(angle->theta3) + 90.0f);
	  osDelay(6000);

    arm->angle = angle->angle;
    arm->theta1 = angle->theta1;
    arm->theta2 = angle->theta2;
    arm->theta3 = angle->theta3;
    if(angle->theta1 > 0) arm->status = A_LEFT;
    else arm->status = A_RIGHT;
}

/**
 * 计算两个位置点之间的欧几里得距离
 * @param tar 目标位置指针
 * @param cur 当前位置指针
 * @return 两点之间的距离误差
 */
float testError(const V_Position* const tar, const Vf_Position* const cur)
{
    // 检查输入参数是否有效
    if (tar == NULL || cur == NULL) {
        return -1.0f; // 返回负值表示错误
    }

    // 计算x、y、z三个方向的差值
    float dx = (float)tar->x - (float)cur->x;
    float dy = (float)tar->y - (float)cur->y;
    float dz = (float)tar->z - (float)cur->z;

    // 计算欧几里得距离 sqrt(dx^2 + dy^2 + dz^2)
    float distance = sqrtf(dx*dx + dy*dy + dz*dz);

    return distance;
}

void moveArm(const uint16_t x,const  uint16_t y,const  uint16_t z){
    V_Position movePosition = {x,y,z};
    V_Angle moveAngle;

    // 计算偏转的角度
    bool ret = _inverseKinematics(&Arm, &movePosition, &moveAngle);

    if(!ret){
        arm_init();
        return;
    }

    debug(
            Vf_Position testMove = _forwardKinematics(&Arm, &moveAngle);

            printf("Robot arm status:\n");
            printf("Base angle: %.2f deg\n", _radiansToDegrees(moveAngle.angle));
            printf("Link1 angle: %.2f deg\n", _radiansToDegrees(moveAngle.theta1));
            printf("Link2 angle: %.2f deg\n", _radiansToDegrees(moveAngle.theta2));
            printf("Link3 angle: %.2f deg\n", _radiansToDegrees(moveAngle.theta3));

            float a1 = moveAngle.theta1 + _degreesToRadians(90);
            float a2 = a1 + moveAngle.theta2;
            float a3 = a2 + moveAngle.theta3; // 如果程序正确 这里的a3应该会是一个特定的值
            printf("本地座标系下,三个夹角为: %.2f %.2f %.2f\n",
                   _radiansToDegrees(a1), _radiansToDegrees(a2), _radiansToDegrees(a3));


            printf("正向运动学结果:\n");
            printf("x : %.2f, y : %.2f, z : %.2f\n", testMove.x, testMove.y, testMove.z);
            printf("误差为 : %f\n", testError(&movePosition, &testMove));
            );

    if(Arm.status == A_LEFT && moveAngle.theta1 < 0
            || Arm.status == A_RIGHT && moveAngle.theta1 > 0) arm_init();

    moveArmAngle(&Arm, &moveAngle);
}

void moveArmForRTOS(const uint16_t x,const  uint16_t y,const  uint16_t z){
    V_Position movePosition = {x,y,z};
    V_Angle moveAngle;

    // 计算偏转的角度
    bool ret = _inverseKinematics(&Arm, &movePosition, &moveAngle);

    if(!ret){
        arm_init_forRTOS();
        return;
    }

    debug(
            Vf_Position testMove = _forwardKinematics(&Arm, &moveAngle);

            printf("Robot arm status:\n");
            printf("Base angle: %.2f deg\n", _radiansToDegrees(moveAngle.angle));
            printf("Link1 angle: %.2f deg\n", _radiansToDegrees(moveAngle.theta1));
            printf("Link2 angle: %.2f deg\n", _radiansToDegrees(moveAngle.theta2));
            printf("Link3 angle: %.2f deg\n", _radiansToDegrees(moveAngle.theta3));

            float a1 = moveAngle.theta1 + _degreesToRadians(90);
            float a2 = a1 + moveAngle.theta2;
            float a3 = a2 + moveAngle.theta3; // 如果程序正确 这里的a3应该会是一个特定的值
            printf("本地座标系下,三个夹角为: %.2f %.2f %.2f\n",
                   _radiansToDegrees(a1), _radiansToDegrees(a2), _radiansToDegrees(a3));


            printf("正向运动学结果:\n");
            printf("x : %.2f, y : %.2f, z : %.2f\n", testMove.x, testMove.y, testMove.z);
            printf("误差为 : %f\n", testError(&movePosition, &testMove));
            );

    if(Arm.status == A_LEFT && moveAngle.theta1 < 0
            || Arm.status == A_RIGHT && moveAngle.theta1 > 0) arm_init_forRTOS();

    moveArmAngleForRTOS(&Arm, &moveAngle);
}



#endif /* APP_ENABLE_ROBOTIC_ARM */
