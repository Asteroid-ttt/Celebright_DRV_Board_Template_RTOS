#include "keyboard.h"
#include "main.h"

/* 私有常量 */
#define ROW_NUM  4
#define COL_NUM  4

/* 按键映射表 */
static const char KEY_MAP[ROW_NUM][COL_NUM] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

/* 引脚配置结构体 */
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


/* 私有函数 */
static void vKeyScanTask(void *pvParams);
void Task_APP_test(void *argument);

void Key_StartScanTask(void) {
    xTaskCreate(vKeyScanTask, "KeyScan", 
               configMINIMAL_STACK_SIZE * 2,
               NULL, tskIDLE_PRIORITY + 2, NULL);
}



static void vKeyScanTask(void *pvParams) {
    static TickType_t xLastTick[ROW_NUM][COL_NUM] = {0};
    static uint8_t keyState[ROW_NUM][COL_NUM] = {0};
    char keyChar = 0;
    for(;;) {
				//OLED_ShowNum(0,0,HAL_GPIO_ReadPin(COLS[0].port,COLS[0].pin), 2, 10);
        for(int row=0; row<ROW_NUM; row++) {
            // 扫描当前行
            HAL_GPIO_WritePin(ROWS[row].port, ROWS[row].pin, GPIO_PIN_RESET);
            vTaskDelay(pdMS_TO_TICKS(1)); // 稳定时间
						
            for(int col=0; col<COL_NUM; col++) {
                // 读取列状态
                GPIO_PinState state = HAL_GPIO_ReadPin(COLS[col].port, COLS[col].pin);

                // 状态机处理
                switch(keyState[row][col]) {
                case 0: // 初始状态
                    if(state == GPIO_PIN_RESET) {
                        keyState[row][col] = 1;
                        xLastTick[row][col] = xTaskGetTickCount();
                    }
                    break;
                
                case 1: // 待消抖确认
                    if((xTaskGetTickCount() - xLastTick[row][col]) > DEBOUNCE_TICKS) {
                        if(state == GPIO_PIN_RESET) {
                            keyChar = KEY_MAP[row][col];
                            xQueueSend(queue_keyHandle, &keyChar, 0);
                            keyState[row][col] = 2;
                        } else {
                            keyState[row][col] = 0;
                        }
                    }
                    break;
                
                case 2: // 等待释放
                    if(state == GPIO_PIN_SET) {
                        keyState[row][col] = 0;
                    }
                    break;
                }
            }
		
            // 恢复行电平
            HAL_GPIO_WritePin(ROWS[row].port, ROWS[row].pin, GPIO_PIN_SET);
            taskYIELD();
        }
        vTaskDelay(pdMS_TO_TICKS(KEY_SCAN_INTERVAL_MS));
    }
}


void Task_APP_test(void *argument)
{
    vTaskDelete(NULL);
}

void key_callback1()
{
	xTaskCreate(Task_APP_test, "APP", configMINIMAL_STACK_SIZE * 2, NULL, osPriorityNormal2, NULL);
}
