//AI GENERATED
#include "stm32h7xx_hal.h"
#include "PWM.h"


// PWM全局变量定义
volatile uint16_t pwm_counter = 500;
volatile uint16_t dutyA = 0, dutyB = 0, dutyC = 0, dutyD = 0;

// 设置占空比函数
void Set_PWM_Duty(uint8_t channel, uint16_t duty_percent) {
    uint16_t duty = duty_percent;  
    // 原子操作保护
    //uint32_t primask = __get_PRIMASK();
    //__disable_irq();
    
    switch(channel) {
        case 0: dutyA = duty; break;  // PWMA
        case 1: dutyB = duty; break;  // PWMB
        case 2: dutyC = duty; break;  // PWMC
        case 3: dutyD = duty; break;  // PWMD
    }
    
    //__set_PRIMASK(primask);
}
