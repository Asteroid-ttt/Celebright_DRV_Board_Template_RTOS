/**
 * @file app_rtos_hooks.c
 * @brief FreeRTOS 钩子实现 —— 运行时统计 + 栈溢出/内存失败错误处理
 *
 * 使用 DWT->CYCCNT (480MHz Cortex-M7) 作为运行时统计计数器。
 * DWT 已在 delay_init(480) 中启用，无需额外配置。
 */
#include "app_rtos_hooks.h"

#include "FreeRTOS.h"
#include "task.h"
#include "display_service.h"

/* ========================================================================
 * 运行时统计 — 使用 DWT 周期计数器（已在 delay_init 中启用）
 * ======================================================================== */
void configureTimerForRunTimeStats(void)
{
    /* DWT->CYCCNT 由 delay_init(480) 启用，无需额外配置 */
}

unsigned long getRunTimeCounterValue(void)
{
    return DWT->CYCCNT;
}

/* ========================================================================
 * 栈溢出钩子
 * ======================================================================== */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;

    __disable_irq();
#if APP_ENABLE_DISPLAY
    DisplayService_Clear();
    DisplayService_ShowText(0, 0, "Error!");
    DisplayService_ShowText(0, 16, "Stack Overflow!");
    if (pcTaskName != NULL) { DisplayService_ShowText(0, 32, pcTaskName); }
#endif
    for (;;) {}
}

void vApplicationMallocFailedHook(void)
{
    __disable_irq();
#if APP_ENABLE_DISPLAY
    DisplayService_Clear();
    DisplayService_ShowText(0, 0, "Error!");
    DisplayService_ShowText(0, 16, "Out of memory!");
#endif
    for (;;) {}
}
    for (;;) {}
}

/* ========================================================================
 * 内存分配失败钩子
 * ======================================================================== */
void vApplicationMallocFailedHook(void)
{
    __disable_irq();
    DisplayService_Clear();
    DisplayService_ShowText(0, 0, "Error!");
    DisplayService_ShowText(0, 16, "Out of memory!");
    for (;;) {}
}
