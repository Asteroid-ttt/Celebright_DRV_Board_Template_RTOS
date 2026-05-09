#ifndef __MATRIX_KEY_H
#define __MATRIX_KEY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"

/*按键配置*/
/**参考原理图中相连的引脚**/
#define MATRIX_ROW1_GPIO_Port GPIOE
#define MATRIX_ROW2_GPIO_Port GPIOE
#define MATRIX_ROW3_GPIO_Port GPIOE
#define MATRIX_ROW4_GPIO_Port GPIOE
#define MATRIX_COL1_GPIO_Port GPIOA
#define MATRIX_COL2_GPIO_Port GPIOB
#define MATRIX_COL3_GPIO_Port GPIOC
#define MATRIX_COL4_GPIO_Port GPIOC

#define MATRIX_ROW1_Pin GPIO_PIN_9
#define MATRIX_ROW2_Pin GPIO_PIN_11
#define MATRIX_ROW3_Pin GPIO_PIN_13
#define MATRIX_ROW4_Pin GPIO_PIN_14
#define MATRIX_COL1_Pin GPIO_PIN_10
#define MATRIX_COL2_Pin GPIO_PIN_5
#define MATRIX_COL3_Pin GPIO_PIN_9
#define MATRIX_COL4_Pin GPIO_PIN_8

/* 功能配置 */
#define KEY_SCAN_INTERVAL_MS   5       // 扫描间隔
#define DEBOUNCE_TICKS         pdMS_TO_TICKS(20) // 消抖时间
#define KEY_QUEUE_LENGTH       10      // 队列长度

/* 全局接口 */
extern osMessageQueueId_t queue_keyHandle;  // 字符队列句柄

void Key_Init(void);
void Key_StartScanTask(void);
void key_callback1(void);

#ifdef __cplusplus
}
#endif

#endif /* __MATRIX_KEY_H */

