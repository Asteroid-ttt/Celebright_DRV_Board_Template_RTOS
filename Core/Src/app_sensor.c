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

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        IMU_getYawPitchRoll(ypr);
        IMU_TT_getgyro(motion6);
        /* NOTE: Car_Attitude_Update_Input() is called by AppCarControl_Task
         * (10ms) after fresh encoder data is available. Calling it here with
         * stale encoder values would overwrite the correct attitude. */
    }
}
#endif
