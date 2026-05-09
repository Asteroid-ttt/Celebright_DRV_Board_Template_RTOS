/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AIN1_Pin GPIO_PIN_3
#define AIN1_GPIO_Port GPIOE
#define IMU_CS_Pin GPIO_PIN_4
#define IMU_CS_GPIO_Port GPIOE
#define AIN2_Pin GPIO_PIN_13
#define AIN2_GPIO_Port GPIOC
#define BIN1_Pin GPIO_PIN_0
#define BIN1_GPIO_Port GPIOC
#define BIN2_Pin GPIO_PIN_3
#define BIN2_GPIO_Port GPIOC
#define PWMC_Pin GPIO_PIN_2
#define PWMC_GPIO_Port GPIOA
#define CIN1_Pin GPIO_PIN_4
#define CIN1_GPIO_Port GPIOC
#define CIN2_Pin GPIO_PIN_5
#define CIN2_GPIO_Port GPIOC
#define PWMA_Pin GPIO_PIN_0
#define PWMA_GPIO_Port GPIOB
#define DIN1_Pin GPIO_PIN_1
#define DIN1_GPIO_Port GPIOB
#define DIN2_Pin GPIO_PIN_7
#define DIN2_GPIO_Port GPIOE
#define ROW1_Pin GPIO_PIN_9
#define ROW1_GPIO_Port GPIOE
#define ROW2_Pin GPIO_PIN_11
#define ROW2_GPIO_Port GPIOE
#define ROW3_Pin GPIO_PIN_13
#define ROW3_GPIO_Port GPIOE
#define ROW4_Pin GPIO_PIN_14
#define ROW4_GPIO_Port GPIOE
#define PWMB_Pin GPIO_PIN_10
#define PWMB_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_10
#define OLED_DC_GPIO_Port GPIOD
#define PWMD_Pin GPIO_PIN_14
#define PWMD_GPIO_Port GPIOD
#define User_Pin GPIO_PIN_15
#define User_GPIO_Port GPIOD
#define COL4_Pin GPIO_PIN_8
#define COL4_GPIO_Port GPIOC
#define COL3_Pin GPIO_PIN_9
#define COL3_GPIO_Port GPIOC
#define Laser_Pin GPIO_PIN_8
#define Laser_GPIO_Port GPIOA
#define COL1_Pin GPIO_PIN_10
#define COL1_GPIO_Port GPIOA
#define Laser2_Pin GPIO_PIN_12
#define Laser2_GPIO_Port GPIOC
#define LED0_Pin GPIO_PIN_3
#define LED0_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOD
#define OLED_RST_Pin GPIO_PIN_6
#define OLED_RST_GPIO_Port GPIOD
#define COL2_Pin GPIO_PIN_5
#define COL2_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_1
#define LED3_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  
typedef const int16_t sc16;  
typedef const int8_t sc8;  

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  
typedef __I int16_t vsc16; 
typedef __I int8_t vsc8;   

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  
typedef const uint16_t uc16;  
typedef const uint8_t uc8; 

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  
typedef __I uint16_t vuc16; 
typedef __I uint8_t vuc8;  
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
