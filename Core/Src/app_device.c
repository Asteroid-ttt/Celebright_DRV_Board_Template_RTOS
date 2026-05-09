/**
 * @file app_device.c
 * @brief Device services — display / broadcast / UART / keyboard / music
 *
 * Each function is guarded by APP_ENABLE_* switch in app.h.
 * Set to 0 to disable the module at compile time.
 */
#include "app.h"

/* ========================================================================
 * Display — OLED/LCD (100ms)
 * ======================================================================== */
#if APP_ENABLE_DISPLAY
void AppDisplay_Task(void *argument)
{
    (void)argument;
    for (;;)
    {
        DisplayService_ShowText(0, 0, "Hello World!");
        osDelay(100);
    }
}
#endif

/* ========================================================================
 * Broadcast — Bluetooth broadcast (50ms)
 * ======================================================================== */
#if APP_ENABLE_BROADCAST
void AppBroadcast_Task(void *argument)
{
    (void)argument;
    for (;;)
    {
        char *msg = "HelloFromStation\n";
        HAL_UART_Transmit(&huart4, (uint8_t *)msg, strlen(msg), 100);
        osDelay(50);
    }
}
#endif

/* ========================================================================
 * Uart3Rx — UART3 frame receive (blocks on queue)
 * ======================================================================== */
#if APP_ENABLE_UART_FIFO
void AppUart3Rx_Task(void *argument)
{
    (void)argument;
    char frame[UART_FRAME_MAX_LEN];

    for (;;)
    {
        memset(frame, 0, sizeof(frame));
        if (xQueueReceive(uart3_frame_queue, frame, portMAX_DELAY) == pdPASS)
        {
            /* TODO: parse frame data */
        }
    }
}

/* ========================================================================
 * Uart4Rx — UART4 frame receive (blocks on queue)
 * ======================================================================== */
void AppUart4Rx_Task(void *argument)
{
    (void)argument;
    char frame[UART_FRAME_MAX_LEN];

    for (;;)
    {
        memset(frame, 0, sizeof(frame));
        if (xQueueReceive(uart4_frame_queue, frame, portMAX_DELAY) == pdPASS)
        {
            /* TODO: parse frame data */
        }
    }
}
#endif

/* ========================================================================
 * KeyScan — keyboard scan
 * ======================================================================== */
#if APP_ENABLE_KEYBOARD
void AppKeyScan_Task(void *argument)
{
    (void)argument;
    for (;;)
    {
        osDelay(1);
    }
}
#endif

/* ========================================================================
 * Buzzer — music player (disabled by default)
 * ======================================================================== */
#if APP_ENABLE_MUSIC
void AppBuzzer_Task(void *argument)
{
    (void)argument;
    for (;;)
    {
        /* Enable: PlayNote(M1, 100); */
        osDelay(1);
    }
}
#endif

/* ========================================================================
 * Reserved — placeholder for future extensions
 * ======================================================================== */
void AppReserved_Task(void *argument)
{
    (void)argument;
    for (;;)
    {
        osDelay(50);
    }
}
