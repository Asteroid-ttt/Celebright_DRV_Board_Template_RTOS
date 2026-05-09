#include "uart_fifo.h"
#if APP_ENABLE_UART_FIFO
#include <stdio.h>

extern QueueHandle_t uart3_frame_queue;  // 由主程序创建
extern QueueHandle_t uart4_frame_queue;  // 由主程序创建

static uint8_t rx4_frame_buf[UART_FRAME_MAX_LEN];
static size_t frame4_idx = 0;
static uint8_t rx4_byte;
static uint8_t rx3_frame_buf[UART_FRAME_MAX_LEN];
static size_t frame3_idx = 0;
static uint8_t rx3_byte;

void uart_frame_rx_init(void)
{
		__HAL_UART_CLEAR_OREFLAG(&huart3);
		__HAL_UART_CLEAR_FEFLAG(&huart3);
		__HAL_UART_CLEAR_NEFLAG(&huart3);
		__HAL_UART_CLEAR_PEFLAG(&huart3);
		__HAL_UART_CLEAR_OREFLAG(&huart4);
		__HAL_UART_CLEAR_FEFLAG(&huart4);
		__HAL_UART_CLEAR_NEFLAG(&huart4);
		__HAL_UART_CLEAR_PEFLAG(&huart4);
    frame4_idx = 0;
    memset(rx4_frame_buf, 0, UART_FRAME_MAX_LEN);
    HAL_UART_Receive_IT(&huart4, &rx4_byte, 1);//bt
		frame3_idx = 0;
    memset(rx3_frame_buf, 0, UART_FRAME_MAX_LEN);
    HAL_UART_Receive_IT(&huart3, &rx3_byte, 1);//cat
}
//以上为bt和cat,3cat4bt
//******************//
extern UART_HandleTypeDef huart1;
uint8_t uart_rx_byte; 


uint8_t uart_received_frame[MAX_FRAME_SIZE * 8]; 
uint16_t uart_received_frame_length = 0; 
uint8_t uart_tx_buffer[MAX_FRAME_SIZE + 2];  // ?????
uint16_t uart_tx_length = 0; 

void UART_Init(void) {
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
}


// This function is used to deconstruct the received data frame, 
// removing the frame header and footer, and dividing the middle data into bits.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_3);
    if (huart->Instance == USART1) {
        static uint8_t frame_data[MAX_FRAME_SIZE];  // ??????
        static uint16_t frame_index = 0;
        
        if (uart_rx_byte == FRAME_HEADER) {
            frame_index = 0;  // ??????????
        }

        if (frame_index < sizeof(frame_data)) {
            frame_data[frame_index++] = uart_rx_byte;
        }

        if (uart_rx_byte == FRAME_TAIL) {
            uart_received_frame_length = frame_index * 8;  // ?? bit ??

            for (uint16_t i = 0; i < frame_index; i++) {
                uint8_t byte = frame_data[i];
                for (uint8_t bit = 0; bit < 8; bit++) {
                    uart_received_frame[i * 8 + bit] = (byte >> (7 - bit)) & 0x01;
                }
            }

        }
        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
    }
		if (huart->Instance == USART3) {
			//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			//HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
        if (rx3_byte == '\n' || frame3_idx >= UART_FRAME_MAX_LEN - 1) {
            rx3_frame_buf[frame3_idx] = '\0';  // 构建C字符串
						//printf("ok");
						
            // 发送至队列（建议用静态队列避免分配碎片）
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(uart3_frame_queue, rx3_frame_buf, &xHigherPriorityTaskWoken);
            frame3_idx = 0;
            memset(rx3_frame_buf, 0, UART_FRAME_MAX_LEN);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        } else if (rx3_byte != '\r') {  // 忽略回车
            rx3_frame_buf[frame3_idx++] = rx3_byte;
        }
        // 继续接收下一个字节
        HAL_UART_Receive_IT(&huart3, &rx3_byte, 1);
    }
		if (huart->Instance == UART4) {
			//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        if (rx4_byte == '\n' || frame4_idx >= UART_FRAME_MAX_LEN - 1) {
            rx4_frame_buf[frame4_idx] = '\0';  // 构建C字符串
						//printf("ok");
            // 发送至队列（建议用静态队列避免分配碎片）
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(uart4_frame_queue, rx4_frame_buf, &xHigherPriorityTaskWoken);
            frame4_idx = 0;
            memset(rx4_frame_buf, 0, UART_FRAME_MAX_LEN);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        } else if (rx4_byte != '\r') {  // 忽略回车
            rx4_frame_buf[frame4_idx++] = rx4_byte;
        }
        // 继续接收下一个字节
        HAL_UART_Receive_IT(&huart4, &rx4_byte, 1);
    }
}

void UART_SendFrame(uint8_t *bit_data, uint16_t bit_length) {
    if (bit_length % 8 != 0) {
        printf("Error: Bit length must be a multiple of 8!\n");
        return;
    }

    uint16_t byte_length = bit_length / 8;  

    uart_tx_buffer[0] = FRAME_HEADER;

    for (uint16_t i = 0; i < byte_length; i++) {
        uart_tx_buffer[i + 1] = 0;
        for (uint8_t k = 0; k < 8; k++) {
            uart_tx_buffer[i + 1] |= (bit_data[i * 8 + k] & 0x01) << (7 - k);
        }
    }

    uart_tx_buffer[byte_length + 1] = FRAME_TAIL;

    uart_tx_length = byte_length + 2;

    HAL_UART_Transmit_IT(&huart1, uart_tx_buffer, uart_tx_length);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        printf("UART Transmission Complete!\n");
    }
}

#endif /* APP_ENABLE_UART_FIFO */
