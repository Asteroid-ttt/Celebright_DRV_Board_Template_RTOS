/**
 * @file app_init.c
 * @brief Application initialization task — runs once, then self-deletes
 *
 * Follows the reference template's AppInit_Task pattern:
 *   1. USB device initialization (post-scheduler-start)
 *   2. Set initial car motion command
 *   3. Self-delete to free stack space
 */
#include "app.h"

extern void MX_USB_DEVICE_Init(void);

void AppInit_Task(void *argument)
{
    (void)argument;

    /* USB device initialization MUST be done after scheduler starts.
       CubeMX also inserts this call in LedBlink_Handler (outside USER CODE).
       To avoid duplication, set CubeMX → USB_DEVICE → Platform Settings
       → "USB Initialization Task" to "none". */
    MX_USB_DEVICE_Init();

    /* Car motion is now controlled via CLI (type 'car' for help).
       The old auto-start has been removed. */

    /* TODO: robotic arm init (uncomment when needed) */
    // setup_roboticArm();
    // init_ArmForRTOS();
    // moveArmForRTOS(150, 200, 50);

    /* Self-delete to free stack */
    vTaskDelete(NULL);
}
