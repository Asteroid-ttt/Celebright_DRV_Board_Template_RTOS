#include "app.h"
#include "keyboard.h"
#if APP_ENABLE_KEYBOARD
#include "main.h"

/* Keyboard scanning logic has been moved to app_device.c (AppKeyScan_Task).
   This file is retained for backward compatibility with existing callers. */

void Key_StartScanTask(void) {
    /* No-op: scanning is now handled by AppKeyScan_Task (CubeMX KeyScan task). */
}

void Task_APP_test(void *argument)
{
    (void)argument;
    vTaskDelete(NULL);
}

void key_callback1()
{
    xTaskCreate(Task_APP_test, "APP", configMINIMAL_STACK_SIZE * 2, NULL, osPriorityNormal2, NULL);
}

#endif /* APP_ENABLE_KEYBOARD */
