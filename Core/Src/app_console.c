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

#define CONSOLE_OUTPUT_MAX_LEN  768U
#define CONSOLE_PROMPT          "> "

static char g_cli_output[CONSOLE_OUTPUT_MAX_LEN];
static bool g_cli_registered = false;

static void Console_Write(const char *str);
static void Console_WritePrompt(void);
static void Console_ProcessLine(char *line);
static void Console_RegisterCommands(void);

static BaseType_t CLI_HelpCommand(char *buf, size_t len, const char *cmd);
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
                CDC_Transmit_FS(&ch, 1);  /* echo */
            }
        }
    }
}

static void Console_Write(const char *str)
{
    if (str != NULL)
    {
        uint16_t len = (uint16_t)strlen(str);
        if (len > 0U)
        {
            CDC_Transmit_FS((uint8_t *)str, len);
        }
    }
}

static void Console_WritePrompt(void)
{
    Console_Write("\r\n" CONSOLE_PROMPT);
}

static void Console_RegisterCommands(void)
{
    static const CLI_Command_Definition_t cmds[] = {
        { "help",  "help: Show commands\r\n",              CLI_HelpCommand,          0 },
        { "tasks", "tasks: Task list\r\n",                 CLI_TaskListCommand,       0 },
        { "stats", "stats: Runtime stats\r\n",             CLI_RunTimeStatsCommand,   0 },
        { "sys",   "sys: System summary\r\n",              CLI_SysCommand,            0 },
        { "flash", "flash [mount|umount|format|ls|cat|info]: QSPI Flash\r\n",  CLI_FlashCommand,  -1 },
        { "car",   "car <mm> [deg]: Move car (go/spin)\r\n", CLI_CarCommand,          -1 },
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
    while (*trimmed == ' ') trimmed++;

    if (trimmed[0] == '\0') return;

    g_cli_output[0] = '\0';
    FreeRTOS_CLIProcessCommand(trimmed, g_cli_output, sizeof(g_cli_output));
    Console_Write(g_cli_output);
}

/* ====== CLI Commands ====== */

static BaseType_t CLI_HelpCommand(char *buf, size_t len, const char *cmd)
{
    (void)cmd;
    const char *help =
        "Commands:\r\n"
        "  help            Show help\r\n"
        "  tasks           List all tasks\r\n"
        "  stats           Run-time CPU stats\r\n"
        "  sys             System summary (heap, stack)\r\n"
        "  flash mount     Mount LittleFS on QSPI\r\n"
        "  flash ls        List flash files\r\n"
        "  flash info      Flash info\r\n"
        "  flash format    Format flash (erases all)\r\n"
        "  car 1000 0      Go straight 1000mm\r\n"
        "  car 0 180       Spin 180 degrees\r\n";
    (void)snprintf(buf, len, "%s", help);
    return pdFALSE;
}

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
    const char *p1 = FreeRTOS_CLIGetParameter(cmd, 1, NULL);
    const char *p2 = FreeRTOS_CLIGetParameter(cmd, 2, NULL);

    if (p1 == NULL)
    {
        (void)snprintf(buf, len, "Usage: car <mm> [deg]\r\n  car 1000 0   = straight 1m\r\n  car 0 180    = spin 180\r\n");
        return pdFALSE;
    }

    float x = (float)atof(p1);
    float angle = (p2 != NULL) ? (float)atof(p2) : 0.0f;

    Set_Car_Control(x, 0, angle);
    (void)snprintf(buf, len, "Car: x=%.1f mm, angle=%.1f deg\r\n", (double)x, (double)angle);
    return pdFALSE;
}

#endif /* APP_ENABLE_CONSOLE */
