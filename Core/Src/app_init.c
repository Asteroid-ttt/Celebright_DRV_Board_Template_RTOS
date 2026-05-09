/**
 * @file app_init.c
 * @brief 应用初始化任务 —— 上电后执行一次，然后自删除
 *
 * 参照 freeRTOS Template 的 AppInit_Task 模式：
 *   1. 执行后调度器启动后的初始化操作
 *   2. 设置小车初始运动命令
 *   3. 自删除释放栈空间
 */
#include "app.h"

void AppInit_Task(void *argument)
{
    (void)argument;

    /* 小车直行 1000mm 后停止 */
    Set_Car_Control(1000, 0, 0);
    osDelay(3000);

    /* TODO: 机械臂初始化（需要时取消注释） */
    // setup_roboticArm();
    // init_ArmForRTOS();
    // moveArmForRTOS(150, 200, 50);

    /* 任务自删除 */
    vTaskDelete(NULL);
}
