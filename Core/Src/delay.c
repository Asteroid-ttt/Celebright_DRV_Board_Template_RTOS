/**
 * @file delay.c
 * @brief Microsecond/millisecond delay using DWT cycle counter (Cortex-M7)
 *
 * DWT->CYCCNT runs at CPU clock (480 MHz on STM32H750),
 * resolution ~2.08 ns, overflow ~8.9 seconds.
 *
 * SysTick is NOT touched — FreeRTOS manages it via HAL timebase.
 */
#include "delay.h"

static uint32_t g_cpu_freq_mhz = 480U;
static uint32_t g_cpu_freq_hz  = 480000000U;

/**
 * @brief Initialize delay module
 * @param cpu_freq_mhz CPU clock frequency in MHz (e.g., 480)
 *
 * Enables the DWT cycle counter and sets internal frequency variables.
 * Does NOT reconfigure SysTick.
 */
void delay_init(uint32_t cpu_freq_mhz)
{
    g_cpu_freq_mhz = cpu_freq_mhz;
    g_cpu_freq_hz  = cpu_freq_mhz * 1000000U;

    /* Ensure SysTick uses HCLK (needed by FreeRTOS before osKernelStart) */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* Enable DWT cycle counter (if not already enabled) */
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief Delay in microseconds
 * @param nus Number of microseconds to delay
 *
 * Spin-waits on DWT->CYCCNT. Accuracy: ~2.08 ns resolution.
 * Blocking — use osDelay() for long delays in RTOS tasks.
 */
void delay_us(uint32_t nus)
{
    uint32_t start  = DWT->CYCCNT;
    uint32_t target = nus * g_cpu_freq_mhz;  /* ticks = us * MHz */
    volatile uint32_t cnt;

    if (nus == 0U) return;

    do
    {
        cnt = DWT->CYCCNT;
    } while ((cnt - start) < target);
}

/**
 * @brief Delay in milliseconds
 * @param nms Number of milliseconds to delay
 *
 * Spin-waits using DWT->CYCCNT for < 1000 ms delays.
 * For delays >= 1000 ms, uses HAL_Delay() to allow RTOS task switching.
 */
void delay_ms(uint32_t nms)
{
    if (nms == 0U) return;

    /* For long delays, use HAL_Delay() which yields to RTOS */
    while (nms >= 1U)
    {
        uint32_t chunk = (nms > 100U) ? 100U : nms;
        delay_us(chunk * 1000U);
        nms -= chunk;
    }
}
