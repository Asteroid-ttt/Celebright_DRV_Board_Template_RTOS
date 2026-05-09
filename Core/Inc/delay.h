/**
 * @file delay.h
 * @brief Microsecond/millisecond delay using DWT cycle counter (Cortex-M7)
 *
 * This module is designed for FreeRTOS environments.
 * It uses the DWT->CYCCNT 32-bit cycle counter (480MHz on H750),
 * which provides ~2.08ns resolution and ~8.9s overflow period.
 *
 * SysTick is managed exclusively by FreeRTOS — this module does NOT
 * reconfigure SysTick in any way.
 */
#ifndef _DELAY_H
#define _DELAY_H
#include "main.h"

void delay_init(uint32_t cpu_freq_mhz);
void delay_ms(uint32_t nms);
void delay_us(uint32_t nus);

#endif /* _DELAY_H */
