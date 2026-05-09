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

#define ROW_NUM  4
#define COL_NUM  4

static const char KEY_MAP[ROW_NUM][COL_NUM] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

static const struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} ROWS[ROW_NUM] = {
    {MATRIX_ROW1_GPIO_Port, MATRIX_ROW1_Pin},
    {MATRIX_ROW2_GPIO_Port, MATRIX_ROW2_Pin},
    {MATRIX_ROW3_GPIO_Port, MATRIX_ROW3_Pin},
    {MATRIX_ROW4_GPIO_Port, MATRIX_ROW4_Pin}
};

static const struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} COLS[COL_NUM] = {
    {MATRIX_COL1_GPIO_Port, MATRIX_COL1_Pin},
    {MATRIX_COL2_GPIO_Port, MATRIX_COL2_Pin},
    {MATRIX_COL3_GPIO_Port, MATRIX_COL3_Pin},
    {MATRIX_COL4_GPIO_Port, MATRIX_COL4_Pin}
};

void AppKeyScan_Task(void *argument)
{
    (void)argument;
    static TickType_t xLastTick[ROW_NUM][COL_NUM] = {0};
    static uint8_t keyState[ROW_NUM][COL_NUM] = {0};
    char keyChar = 0;

    for (;;)
    {
        for (int row = 0; row < ROW_NUM; row++)
        {
            HAL_GPIO_WritePin(ROWS[row].port, ROWS[row].pin, GPIO_PIN_RESET);
            vTaskDelay(pdMS_TO_TICKS(1));

            for (int col = 0; col < COL_NUM; col++)
            {
                GPIO_PinState state = HAL_GPIO_ReadPin(COLS[col].port, COLS[col].pin);

                switch (keyState[row][col])
                {
                case 0:
                    if (state == GPIO_PIN_RESET)
                    {
                        keyState[row][col] = 1;
                        xLastTick[row][col] = xTaskGetTickCount();
                    }
                    break;

                case 1:
                    if ((xTaskGetTickCount() - xLastTick[row][col]) > DEBOUNCE_TICKS)
                    {
                        if (state == GPIO_PIN_RESET)
                        {
                            keyChar = KEY_MAP[row][col];
                            xQueueSend(queue_keyHandle, &keyChar, 0);
                            keyState[row][col] = 2;
                        }
                        else
                        {
                            keyState[row][col] = 0;
                        }
                    }
                    break;

                case 2:
                    if (state == GPIO_PIN_SET)
                    {
                        keyState[row][col] = 0;
                    }
                    break;
                }
            }

            HAL_GPIO_WritePin(ROWS[row].port, ROWS[row].pin, GPIO_PIN_SET);
            taskYIELD();
        }
        vTaskDelay(pdMS_TO_TICKS(KEY_SCAN_INTERVAL_MS));
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
