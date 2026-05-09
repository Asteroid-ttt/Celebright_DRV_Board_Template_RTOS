#ifndef __UART_FIFO_H
#define __UART_FIFO_H

#include "stm32h7xx_hal.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>

#ifndef __UART_FRAME_RX_H
#define __UART_FRAME_RX_H

#include <stdint.h>
#include <stddef.h>

// 队列每帧最大长度
#define UART_FRAME_MAX_LEN 64

// 初始化串口接收帧机制
void uart_frame_rx_init(void);

#endif

// ?????
#define FRAME_HEADER         0xAA  // ??
#define FRAME_TAIL           0x55  // ??
#define MAX_FRAME_SIZE       256   // ?????(???)

// ????
extern UART_HandleTypeDef huart1;

extern uint8_t uart_rx_byte;  // ?? UART ?? FIFO ???????
extern uint8_t uart_received_frame[MAX_FRAME_SIZE * 8];  // ?? bit ??
extern uint16_t uart_received_frame_length;  // ?????? bit ??
extern uint8_t uart_tx_buffer[MAX_FRAME_SIZE + 2];  // ???/?????
extern uint16_t uart_tx_length;  // ???????


void UART_Init(void);
void UART_SendFrame(uint8_t *bit_data, uint16_t bit_length);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

#endif // __UART_FIFO_H
