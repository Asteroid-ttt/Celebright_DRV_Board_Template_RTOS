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
 * Runtime stats — DWT cycle counter (480 MHz), scaled down to avoid
 * frequent 32-bit wrap.  Raw DWT wraps every ~8.9s at 480 MHz,
 * corrupting accumulated stats.  Shifting by 8 yields 1.875 MHz →
 * wraps every ~38 minutes, sufficient for interactive use.
 * ======================================================================== */
void configureTimerForRunTimeStats(void)
{
    /* DWT->CYCCNT is enabled in delay_init(480); no extra setup needed */
}

unsigned long getRunTimeCounterValue(void)
{
    /* Downscale to prevent 32-bit wrap every 8.9 seconds.
       At 480 MHz / 256 = 1.875 MHz → wrap every ~2285 seconds (~38 min). */
    return DWT->CYCCNT >> 8;
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
/* ========================================================================
 * 内存分配失败钩子
 * ======================================================================== */
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
