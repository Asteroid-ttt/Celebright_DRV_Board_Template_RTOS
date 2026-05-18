/**
 * @file app_console.c
 * @brief CLI Console via USB CDC Virtual COM Port
 *
 * Uses FreeRTOS+CLI for command parsing.
 * RX: USB CDC -> StreamBuffer (ISR-safe) -> Console Task
 * TX: Console Task -> CDC_Transmit_FS
 *
 * Commands: help / tasks / stats / sys / flash / car
 */
#include "app_console.h"

#if APP_ENABLE_CONSOLE

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "FreeRTOS_CLI.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

#define CONSOLE_OUTPUT_MAX_LEN  768U
#define CONSOLE_PROMPT          "> "

static char g_cli_output[CONSOLE_OUTPUT_MAX_LEN];
static bool g_cli_registered = false;

static void Console_Write(const char *str);
static void Console_WritePrompt(void);
static void Console_ProcessLine(char *line);
static void Console_RegisterCommands(void);

static BaseType_t CLI_TaskListCommand(char *buf, size_t len, const char *cmd);
static BaseType_t CLI_RunTimeStatsCommand(char *buf, size_t len, const char *cmd);
static BaseType_t CLI_SysCommand(char *buf, size_t len, const char *cmd);
static BaseType_t CLI_FlashCommand(char *buf, size_t len, const char *cmd);
static BaseType_t CLI_CarCommand(char *buf, size_t len, const char *cmd);

void AppConsole_Init(void)
{
    Console_RegisterCommands();
}

void AppConsole_Task(void *argument)
{
    char line_buf[96];
    uint8_t ch;
    size_t rx_idx = 0U;

    (void)argument;

    if (!g_cli_registered)
    {
        Console_RegisterCommands();
    }

    Console_Write("\r\n=== Celebright CLI ===\r\n");
    Console_WritePrompt();

    for (;;)
    {
        /* Block on StreamBuffer, waiting for USB CDC data */
        if (xStreamBufferReceive(console_rx_stream, &ch, 1, portMAX_DELAY) == 1)
        {
            if (ch == '\r' || ch == '\n')
            {
                if (rx_idx > 0U)
                {
                    line_buf[rx_idx] = '\0';
                    Console_Write("\r\n");
                    Console_ProcessLine(line_buf);
                    rx_idx = 0U;
                }
                Console_WritePrompt();
            }
            else if (ch == '\b' || ch == 0x7F)
            {
                if (rx_idx > 0U)
                {
                    rx_idx--;
                    Console_Write("\b \b");
                }
            }
            else if (rx_idx < (sizeof(line_buf) - 1U) && ch >= 0x20)
            {
                line_buf[rx_idx++] = (char)ch;
            }
        }
    }
}

static void Console_Write(const char *str)
{
    USBD_CDC_HandleTypeDef *hcdc;
    uint16_t len;
    volatile int retry;

    if (str == NULL) return;
    len = (uint16_t)strlen(str);
    if (len == 0U) return;

    if (hUsbDeviceFS.pClassData == NULL) return;
    hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

    /* Wait for previous USB IN transfer to complete.  CDC_Transmit_FS
       returns USBD_BUSY if TxState != 0.  The USB TX-complete ISR
       clears TxState after the host ACKs the packet (~50us). */
    for (retry = 0; retry < 500000; retry++)
    {
        if (hcdc->TxState == 0) break;
    }

    if (hcdc->TxState == 0)
    {
        (void)CDC_Transmit_FS((uint8_t *)str, len);
    }
}

static void Console_WritePrompt(void)
{
    Console_Write("\r\n" CONSOLE_PROMPT);
}

static void Console_RegisterCommands(void)
{
    /* The built-in "help" command (prvHelpCommand in FreeRTOS_CLI.c) already
       lists all registered commands and their help strings. Do NOT register a
       custom "help" — it would conflict with the built-in multi-output iterator. */
    static const CLI_Command_Definition_t cmds[] = {
        { "tasks", "tasks: List all tasks\r\n",            CLI_TaskListCommand,       0 },
        { "stats", "stats: Runtime CPU stats\r\n",         CLI_RunTimeStatsCommand,   0 },
        { "sys",   "sys: System summary (heap/stack)\r\n", CLI_SysCommand,            0 },
        { "flash", "flash [mount|umount|format|ls|cat|info]: QSPI Flash\r\n",  CLI_FlashCommand,  -1 },
        { "car",   "car [go|spin|arc|stop|start|speed|status]: Car motion control\r\n", CLI_CarCommand, -1 },
    };

    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++)
    {
        FreeRTOS_CLIRegisterCommand(&cmds[i]);
    }
    g_cli_registered = true;
}

static void Console_ProcessLine(char *line)
{
    char *trimmed = line;
    BaseType_t xMore;
    while (*trimmed == ' ') trimmed++;

    if (trimmed[0] == '\0') return;

    /* FreeRTOS+CLI commands may return pdTRUE to indicate more output
       follows (e.g. the built-in "help" command iterates through all
       registered commands one per call). Loop until pdFALSE. */
    do
    {
        g_cli_output[0] = '\0';
        xMore = FreeRTOS_CLIProcessCommand(trimmed, g_cli_output, sizeof(g_cli_output));
        Console_Write(g_cli_output);
    } while (xMore == pdTRUE);
}

/* ====== CLI Commands ====== */

static BaseType_t CLI_TaskListCommand(char *buf, size_t len, const char *cmd)
{
    (void)cmd;
    buf[0] = '\0';
    vTaskList(buf);
    return pdFALSE;
}

static BaseType_t CLI_RunTimeStatsCommand(char *buf, size_t len, const char *cmd)
{
    (void)cmd;
    buf[0] = '\0';
    vTaskGetRunTimeStats(buf);
    return pdFALSE;
}

static BaseType_t CLI_SysCommand(char *buf, size_t len, const char *cmd)
{
    (void)cmd;
#if APP_ENABLE_SYSMON
    AppSysMon_UpdateSnapshot();
    AppSysMon_BuildSystemSummary(buf, len);
#else
    (void)snprintf(buf, len, "SysMon disabled (APP_ENABLE_SYSMON=0).\r\n");
#endif
    return pdFALSE;
}

static BaseType_t CLI_FlashCommand(char *buf, size_t len, const char *cmd)
{
#if APP_ENABLE_QSPI
    const char *param = FreeRTOS_CLIGetParameter(cmd, 1, NULL);

    if (!AppQSPI_IsPresent())
    {
        (void)snprintf(buf, len, "Flash not detected.\r\n");
        return pdFALSE;
    }

    if (param == NULL || strcmp(param, "info") == 0)
    {
        AppQSPI_BuildInfo(buf, len);
    }
    else if (strcmp(param, "mount") == 0)
    {
        if (AppQSPI_Mount())
            (void)snprintf(buf, len, "Flash mounted (LittleFS).\r\n");
        else
            (void)snprintf(buf, len, "Mount failed.\r\n");
    }
    else if (strcmp(param, "umount") == 0)
    {
        AppQSPI_Umount();
        (void)snprintf(buf, len, "Flash unmounted.\r\n");
    }
    else if (strcmp(param, "format") == 0)
    {
        (void)snprintf(buf, len, "Formatting... ");
        if (AppQSPI_Format())
            (void)snprintf(buf + strlen(buf), len - strlen(buf), "OK\r\n");
        else
            (void)snprintf(buf + strlen(buf), len - strlen(buf), "FAILED\r\n");
    }
    else if (strcmp(param, "ls") == 0)
    {
        if (AppQSPI_IsMounted())
            AppQSPI_ListDir(buf, len);
        else
            (void)snprintf(buf, len, "Flash not mounted. Use 'flash mount' first.\r\n");
    }
    else if (strcmp(param, "cat") == 0)
    {
        const char *path = FreeRTOS_CLIGetParameter(cmd, 2, NULL);
        if (path && AppQSPI_IsMounted())
            AppQSPI_ReadFile(buf, len, path, 256U);
        else
            (void)snprintf(buf, len, "Usage: flash cat <filename>\r\n");
    }
    else
    {
        (void)snprintf(buf, len, "flash [mount|umount|format|ls|cat|info]\r\n");
    }
#else
    (void)snprintf(buf, len, "QSPI Flash disabled (APP_ENABLE_QSPI=0).\r\n");
#endif
    return pdFALSE;
}

static BaseType_t CLI_CarCommand(char *buf, size_t len, const char *cmd)
{
    const char *sub = FreeRTOS_CLIGetParameter(cmd, 1, NULL);
    const char *p1  = FreeRTOS_CLIGetParameter(cmd, 2, NULL);
    const char *p2  = FreeRTOS_CLIGetParameter(cmd, 3, NULL);
    float v, x, angle;

    if (sub == NULL || strcmp(sub, "help") == 0)
    {
        (void)snprintf(buf, len,
            "car commands:\r\n"
            "  car go <mm>          Go straight N mm\r\n"
            "  car spin <deg>        Spin N degrees\r\n"
            "  car arc <mm> <deg>    Move N mm while turning M deg\r\n"
            "  car stop              Stop immediately\r\n"
            "  car start             Resume after stop\r\n"
            "  car speed <mm/s>      Set raw speed (0 = stop)\r\n"
            "  car status            Show current state (yaw/speed)\r\n");
        return pdFALSE;
    }

    if (strcmp(sub, "stop") == 0)
    {
        Set_Car_Control(0, 0, 0);
        (void)snprintf(buf, len, "Car: stopped.\r\n");
    }
    else if (strcmp(sub, "start") == 0)
    {
        Set_Car_Start();
        (void)snprintf(buf, len, "Car: resumed.\r\n");
    }
    else if (strcmp(sub, "status") == 0)
    {
        (void)snprintf(buf, len,
            "--- Car Status ---\r\n"
            "Yaw:   %6.1f deg (total %7.1f, circles %d)\r\n"
            "V_Lin: %6.1f / %6.1f mm/s  (cur / tgt)\r\n"
            "V_Ang: %6.1f / %6.1f deg/s (cur / tgt)\r\n"
            "Stop:  %s\r\n",
            (double)car_state.yaw, (double)car_state.yaw_all, car_state.yaw_circles,
            (double)car_state.v_line, (double)car_state.v_line_target,
            (double)car_state.v_angle, (double)car_state.v_angle_target,
            car_state.flag_stop ? "YES" : "NO");
    }
    else if (strcmp(sub, "speed") == 0)
    {
        if (p1 == NULL)
        { (void)snprintf(buf, len, "Usage: car speed <mm/s>\r\n"); return pdFALSE; }
        v = (float)atof(p1);
        Set_Car_Attitude(v, 0);
        (void)snprintf(buf, len, "Car speed: %.1f mm/s\r\n", (double)v);
    }
    else if (strcmp(sub, "go") == 0)
    {
        if (p1 == NULL)
        { (void)snprintf(buf, len, "Usage: car go <mm>\r\n"); return pdFALSE; }
        x = (float)atof(p1);
        Set_Car_Control(x, 0, 0);
        (void)snprintf(buf, len, "Car: go %.1f mm\r\n", (double)x);
    }
    else if (strcmp(sub, "spin") == 0)
    {
        if (p1 == NULL)
        { (void)snprintf(buf, len, "Usage: car spin <deg>\r\n"); return pdFALSE; }
        angle = (float)atof(p1);
        Set_Car_Control(0, 0, angle);
        (void)snprintf(buf, len, "Car: spin %.1f deg\r\n", (double)angle);
    }
    else if (strcmp(sub, "arc") == 0)
    {
        if (p1 == NULL || p2 == NULL)
        { (void)snprintf(buf, len, "Usage: car arc <mm> <deg>\r\n"); return pdFALSE; }
        x = (float)atof(p1);
        angle = (float)atof(p2);
        Set_Car_Control(x, 0, angle);
        (void)snprintf(buf, len, "Car: go %.1f mm, spin %.1f deg\r\n", (double)x, (double)angle);
    }
    else
    {
        (void)snprintf(buf, len, "Unknown: car %s.  Type 'car' for help.\r\n", sub);
    }
    return pdFALSE;
}

#endif /* APP_ENABLE_CONSOLE */
