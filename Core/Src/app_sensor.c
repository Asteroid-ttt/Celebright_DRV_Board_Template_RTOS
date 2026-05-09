/**
 * @file app_sensor.c
 * @brief Sensor service — 5ms IMU data acquisition and attitude update
 */
#include "app.h"

#if APP_ENABLE_IMU
void AppIMUService_Task(void *argument)
{
    (void)argument;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = TASK_ITV_IMU;

    float ypr_local[3];
    float motion6_local[7];

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        IMU_getYawPitchRoll(ypr_local);
        IMU_TT_getgyro(motion6_local);
        Car_Attitude_Update_Input();
    }
}
#endif
