/**
 * @file app_sysmon.c
 * @brief 系统监控任务 —— 堆剩余、任务栈水位、运行时统计
 *
 * 每 1 秒采集快照，每 200ms 更新显示。
 * 可通过 CLI `sys` / `stats` 命令查询。
 */
#include "app_sysmon.h"

#if APP_ENABLE_SYSMON

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "display_service.h"

#define SYSMON_SNAPSHOT_PERIOD_MS 1000U
#define SYSMON_DISPLAY_PERIOD_MS  200U
#define RUNTIME_STATS_BUFFER_SIZE 768U

static volatile uint32_t    g_heap_free = 0U;
static volatile UBaseType_t g_wm_display = 0U;
static volatile UBaseType_t g_wm_carcontrol = 0U;
static volatile UBaseType_t g_wm_imuservice = 0U;
static volatile UBaseType_t g_wm_console = 0U;

extern osThreadId_t DisplayHandle;
extern osThreadId_t CarControlHandle;
extern osThreadId_t IMUServiceHandle;
extern osThreadId_t ConsoleHandle;

void AppSysMon_UpdateSnapshot(void)
{
    g_heap_free = xPortGetFreeHeapSize();
    g_wm_display    = DisplayHandle    ? uxTaskGetStackHighWaterMark(DisplayHandle)    : 0U;
    g_wm_carcontrol = CarControlHandle ? uxTaskGetStackHighWaterMark(CarControlHandle) : 0U;
    g_wm_imuservice = IMUServiceHandle ? uxTaskGetStackHighWaterMark(IMUServiceHandle) : 0U;
    g_wm_console    = ConsoleHandle    ? uxTaskGetStackHighWaterMark(ConsoleHandle)    : 0U;
}

size_t AppSysMon_BuildSystemSummary(char *buf, size_t buf_len)
{
    size_t used;

    if ((buf == NULL) || (buf_len == 0U)) return 0U;

    used = (size_t)snprintf(buf, buf_len,
        "=== System Summary ===\r\n"
        "Heap Free: %lu bytes\r\n"
        "Stack WM (words): Display=%lu CarCtrl=%lu IMU=%lu Console=%lu\r\n",
        (unsigned long)g_heap_free,
        (unsigned long)g_wm_display, (unsigned long)g_wm_carcontrol,
        (unsigned long)g_wm_imuservice, (unsigned long)g_wm_console);

    return (used < buf_len) ? used : (buf_len - 1U);
}

size_t AppSysMon_BuildRuntimeStats(char *buf, size_t buf_len)
{
    if ((buf == NULL) || (buf_len == 0U)) return 0U;
    buf[0] = '\0';
    vTaskGetRunTimeStats(buf);
    return strlen(buf);
}

void AppSysMon_Task(void *argument)
{
    uint32_t last_snapshot_tick = 0U;
    char line_buf[64];

    (void)argument;

    for (;;)
    {
        uint32_t now = osKernelGetTickCount();

        if ((now - last_snapshot_tick) >= SYSMON_SNAPSHOT_PERIOD_MS)
        {
            last_snapshot_tick = now;
            AppSysMon_UpdateSnapshot();
        }

#if APP_ENABLE_DISPLAY
        /* Show heap free + stack watermarks on OLED */
        (void)snprintf(line_buf, sizeof(line_buf), "Heap:%luB", (unsigned long)g_heap_free);
        DisplayService_ShowText(0, 0, line_buf);

        (void)snprintf(line_buf, sizeof(line_buf), "D:%lu C:%lu", (unsigned long)g_wm_display, (unsigned long)g_wm_carcontrol);
        DisplayService_ShowText(0, 16, line_buf);

        (void)snprintf(line_buf, sizeof(line_buf), "I:%lu K:%lu", (unsigned long)g_wm_imuservice, (unsigned long)g_wm_console);
        DisplayService_ShowText(0, 32, line_buf);
#endif

        osDelay(SYSMON_DISPLAY_PERIOD_MS);
    }
}

#endif /* APP_ENABLE_SYSMON */
