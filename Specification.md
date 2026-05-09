# CELEBRIGHT CAR CONTROL BOARD — Full System Specification

**MCU:** STM32H750VBT6 (Cortex-M7 @ 480 MHz)
**RTOS:** FreeRTOS V10.3.1 via CMSIS-OS2 wrapper
**Toolchain:** STM32CubeMX + MDK-ARM (Keil)
**Board:** Celebright Car Control Board (4-wheel drive platform)
**Date:** 2026-05-09

---

## Table of Contents

1. [app.h — Facade Header](#1-apph--facade-header)
2. [config.h — Configuration Aggregator](#2-configh--configuration-aggregator)
3. [main.c — Entry Point](#3-mainc--entry-point)
4. [freertos.c — RTOS Configuration](#4-freertosc--rtos-configuration)
5. [FreeRTOSConfig.h — Kernel Configuration](#5-freertosconfigh--kernel-configuration)
6. [Configuration Files](#6-configuration-files)
   - [6.1 platform_config.h](#61-platform_configh--platform-constants)
   - [6.2 motor_config.h](#62-motor_configh--motor-pid-parameters)
   - [6.3 control_config.h](#63-control_configh--control-pid-parameters)
   - [6.4 rtos_config.h](#64-rtos_configh--rtos-task-timing)
7. [App Service Layer](#7-app-service-layer)
   - [7.1 app_init.c](#71-app_initc--init-task)
   - [7.2 app_control.c](#72-app_controlc--car-control-task)
   - [7.3 app_sensor.c](#73-app_sensorc--sensor-task)
   - [7.4 app_device.c](#74-app_devicec--device-tasks)
   - [7.5 app_sysmon.c](#75-app_sysmonc--system-monitor)
   - [7.6 app_console.c](#76-app_consolec--cli-console)
   - [7.7 app_qspi.c](#77-app_qspic--qspi-flash-service)
   - [7.8 app_rtos_hooks.c](#78-app_rtos_hooksc--rtos-hooks)
   - [7.9 display_service.c](#79-display_servicec--display-abstraction)
8. [Control Layer](#8-control-layer)
   - [8.1 pid.h / pid.c](#81-pidh--pidc--pid-controller)
   - [8.2 PWM.h / PWM.c](#82-pwmh--pwmc--software-pwm)
   - [8.3 Motor.h / Motor.c](#83-motorh--motorc--motor-driver)
   - [8.4 car_attitude.h / car_attitude.c](#84-car_attitudeh--car_attitudec--car-attitude)
   - [8.5 car_control.h / car_control.c](#85-car_controlh--car_controlc--car-control)
9. [Sensor Layer](#9-sensor-layer)
   - [9.1 IMU.h / IMU.c](#91-imuh--imuc--imu-driver--ahrs)
   - [9.2 read_aux_data_mode.h / read_aux_data_mode.c](#92-read_aux_data_modeh--read_aux_data_modec--imu-transport)
10. [Common Layer](#10-common-layer)
    - [10.1 delay.h / delay.c](#101-delayh--delayc--dwt-delay)
    - [10.2 sys_time.h / sys_time.c](#102-sys_timeh--sys_timec--system-time)
11. [Device Layer](#11-device-layer)
    - [11.1 uart_fifo.h / uart_fifo.c](#111-uart_fifoh--uart_fifoc--uart-frame-protocol)
    - [11.2 keyboard.h / keyboard.c](#112-keyboardh--keyboardc--matrix-keyboard)
12. [Storage Layer](#12-storage-layer)
    - [12.1 w25q256.h / w25q256.c](#121-w25q256h--w25q256c--qspi-flash-driver)
    - [12.2 lfs_port.h / lfs_port.c](#122-lfs_porth--lfs_portc--littlefs-port)
13. [Interrupt Handlers](#13-interrupt-handlers)
    - [13.1 stm32h7xx_it.c](#131-stm32h7xx_itc--isr-implementation)
    - [13.2 stm32h7xx_hal_msp.c](#132-stm32h7xx_hal_mspc--msp-initialization)
14. [App Thin-Header Wrappers](#14-app-thin-header-wrappers)
15. [Complete FreeRTOS Task Table](#15-complete-freertos-task-table)
16. [Interrupt Vector Table](#16-interrupt-vector-table)
17. [Pin Map](#17-pin-map)
18. [Memory Layout](#18-memory-layout)
19. [Build Instructions](#19-build-instructions)
20. [CubeMX Regeneration Checklist](#20-cubemx-regeneration-checklist)

---

## 1. app.h — Facade Header

**Path:** `Core/Inc/app.h` | **Lines:** 1-257
**Purpose:** Single-entry-point facade header. All CubeMX-generated `.c` files include only this header. Provides cross-module type declarations, shared variables, compile-time module switches, and unified function prototypes.

### Line-by-Line Annotation

```
Line 1:    /**                                     — Doxygen file comment start
Line 2:     * @file app.h                           — File identifier
Line 3:     * @brief Unified Facade header          — Description
Line 4-6:   Architecture principle comments
Line 7-8:   Core architecture principle: high cohesion, low coupling
Line 9-12:  Include rules documentation:
              CubeMX-generated .c files → ONLY #include "app.h"
              User module .c files     → MAY #include "app.h" or individual headers
              Cross-module shared types/variables → declared in app.h
Line 14-16: Module Trimming documentation:
              Set APP_ENABLE_* to 0 to remove module code from build (saves FLASH)
Line 17:    For maximum RAM savings, also delete the corresponding task in CubeMX
Line 18:    */                                      — Doxygen comment close
Line 20:    #ifndef __APP_H__                      — Include guard: prevents multiple inclusion
Line 21:    #define __APP_H__                      — Include guard: defines the guard symbol
Line 23:    #ifdef __cplusplus                     — C++ compatibility: if C++ compiler
Line 24:    extern "C" {                           — Use C linkage for function names
Line 25:    #endif                                  — End C++ check
Line 27-37: Module Enable Switches documentation block:
Line 38:    #define APP_ENABLE_SYSMON       1       — System monitor: heap free + stack watermarks
Line 39:    #define APP_ENABLE_CONSOLE      1       — CLI console via USB CDC Virtual COM Port
Line 40:    #define APP_ENABLE_QSPI         1       — QSPI Flash (W25Q256) + LittleFS filesystem
Line 41:    #define APP_ENABLE_DISPLAY      1       — OLED/LCD display service
Line 42:    #define APP_ENABLE_KEYBOARD     1       — 4x4 matrix keyboard scanner
Line 43:    #define APP_ENABLE_MUSIC        0       — Buzzer music player (DISABLED by default)
Line 44:    #define APP_ENABLE_UART_FIFO    1       — UART3/UART4 line-based frame receive (ISR→queue)
Line 45:    #define APP_ENABLE_ROBOTIC_ARM  0       — Robotic arm (SCServo, ARobot) (DISABLED)
Line 46:    #define APP_ENABLE_IMU          1       — IMU sensor ICM-45686 (SPI4) + Madgwick AHRS
Line 47:    #define APP_ENABLE_LEDBLINK     1       — LED heartbeat blink (1Hz toggle)
Line 48:    #define APP_ENABLE_BROADCAST    0       — Bluetooth broadcast via UART4 (DISABLED)
Line 50:    #define DISPLAY_TYPE_OLED  0             — Display type constant: OLED (SSD1306)
Line 51:    #define DISPLAY_TYPE_LCD   1             — Display type constant: LCD (ST7789)
Line 52:    #define APP_DISPLAY_TYPE   DISPLAY_TYPE_OLED — Currently configured for OLED
Line 55-60: Common Layer includes (always included):
Line 58:    #include "config.h"                      — Aggregates platform/motor/control/rtos configs
Line 59:    #include "delay.h"                       — DWT-based us/ms delay (no SysTick usage)
Line 60:    #include "sys_time.h"                    — System time (nowtime, HAL_GetTick wrapper)
Line 62-70: Control Layer includes (ALWAYS enabled — core system):
Line 66:    #include "pid.h"                         — Incremental & positional PID controller
Line 67:    #include "PWM.h"                         — 4-channel software PWM via TIM6 ISR
Line 68:    #include "Motor.h"                       — Motor driver (encoder, speed PID, H-bridge)
Line 69:    #include "car_attitude.h"                — Car attitude: v_line, v_angle, yaw tracking
Line 70:    #include "car_control.h"                 — High-level control: GO_LINE, TO_POINT, SPIN
Line 72-83: Sensor Layer includes (guarded by APP_ENABLE_IMU):
Line 76:    #include "inv_imu_defs.h"                — InvenSense IMU driver constants/structures
Line 77:    #include "inv_imu_version.h"             — InvenSense IMU driver version info
Line 78:    #include "inv_imu_transport.h"           — InvenSense transport interface (read/write/sleep)
Line 79:    #include "inv_imu.h"                     — InvenSense IMU core API (register ops)
Line 80:    #include "inv_imu_driver.h"              — InvenSense ICM-45686 device-specific driver
Line 81:    #include "read_aux_data_mode.h"          — SPI4 transport layer for ICM-45686
Line 82:    #include "IMU.h"                         — App-level IMU: Madgwick AHRS, yaw/pitch/roll
Line 83:    #endif
Line 85-91: Device Layer includes (each guarded by APP_ENABLE_*):
Line 93:    #if APP_ENABLE_UART_FIFO → #include "uart_fifo.h" — UART3/UART4 frame protocol
Line 96:    #if APP_ENABLE_DISPLAY
Line 97:    #include "oled_spi_V0.2.h"               — SSD1306 OLED driver (SPI interface)
Line 98:    #include "lcd_st7789.h"                  — ST7789 TFT LCD driver (SPI interface)
Line 99:    #include "lcd_model.h"                   — LCD display model abstraction layer
Line 102:   #if APP_ENABLE_KEYBOARD → #include "keyboard.h" — 4x4 matrix key scanner
Line 106:   #if APP_ENABLE_MUSIC → #include "MusicPlayer.h" — Buzzer music player
Line 110-119: Robot Layer includes (guarded by APP_ENABLE_ROBOTIC_ARM):
Line 114:   #include "SCServo.h"                     — FT SCServo serial protocol library
Line 115:   #include "arm.h"                         — Robotic arm configuration
Line 116:   #include "roboticArm.h"                  — Robotic arm abstraction
Line 117:   #include "ARobot.h"                      — Advanced robot arm control
Line 118:   #include "holder2D.h"                    — 2D holder/gripper control
Line 121-153: Cross-module Shared Data (extern declarations):
Line 126-130: #if APP_ENABLE_IMU:
Line 127:   extern float ypr[3];                     — Yaw[0], Pitch[1], Roll[2] (deg), defined main.c:60
Line 128:   extern float motion6[7];                — Accel[0..2] + Gyro[3..5] + Temp[6], defined main.c:59
Line 129:   extern float imu_g_z;                  — Z-axis gyro angular rate (deg/s), defined IMU.c:28
Line 131:   extern int math_pl;                    — Math placeholder integer, defined main.c:61
Line 133-138: Software PWM duty variables (defined in PWM.c:7-8):
Line 134:   extern volatile uint16_t dutyA;          — PWM Channel 0 duty count (Left Front)
Line 135:   extern volatile uint16_t dutyB;          — PWM Channel 1 duty count (Left Rear)
Line 136:   extern volatile uint16_t dutyC;          — PWM Channel 2 duty count (Right Rear)
Line 137:   extern volatile uint16_t dutyD;          — PWM Channel 3 duty count (Right Front)
Line 138:   extern volatile uint16_t pwm_counter;    — 500-count down-counter (20ms PWM period)
Line 140-153: FreeRTOS cross-module handles:
Line 142-143: #if APP_ENABLE_UART_FIFO:
              extern QueueHandle_t uart3_frame_queue; — 5-slot × 64B, defined freertos.c:53
              extern QueueHandle_t uart4_frame_queue; — 5-slot × 64B, defined freertos.c:54
Line 146:   extern osMessageQueueId_t queue_keyHandle;  — 1-slot char queue, defined freertos.c:205
Line 150-152: #if APP_ENABLE_CONSOLE:
              #include "stream_buffer.h"               — FreeRTOS StreamBuffer for byte-level RX
              extern StreamBufferHandle_t console_rx_stream; — 256-byte stream, defined freertos.c:56
Line 155-163: Task handles for SysMon watermark monitoring:
Line 157-159: extern osThreadId_t DisplayHandle;    — freertos.c:145
             extern osThreadId_t CarControlHandle; — freertos.c:109
             extern osThreadId_t IMUServiceHandle; — freertos.c:97
Line 162:   extern osThreadId_t ConsoleHandle;    — freertos.c:193
Line 167-184: Cross-module Shared Type: car_state_t:
Line 173:   typedef struct {
Line 174:     float yaw;                         — Current yaw angle [0, 360) degrees
Line 175:     float yaw_all;                     — Accumulated yaw including multi-circle overflow
Line 176:     int   yaw_circles;                — Circle overflow count; positive = CCW (360→0)
Line 177:     float v_line;                     — Current linear velocity (mm/s)
Line 178:     float v_angle;                    — Current angular velocity (deg/s)
Line 179:     float v_line_target;              — Target linear velocity (mm/s)
Line 180:     float v_angle_target;             — Target angular velocity (deg/s)
Line 181:     _Bool flag_stop;                  — Car stopped flag (1 = stopped)
Line 182:   } car_state_t;
Line 184:   extern car_state_t car_state;              — Defined car_attitude.c:19
Line 186-194: Display Abstraction Layer:
Line 190:   void DisplayService_Init(void);             — Init OLED or LCD depending on APP_DISPLAY_TYPE
Line 191:   void DisplayService_Clear(void);            — Clear entire display
Line 192:   void DisplayService_ShowText(x, y, text);   — Display ASCII string at pixel coords
Line 193:   void DisplayService_ShowNumber(x, y, n, l); — Display signed integer with sign handling
Line 196-229: FreeRTOS Task Service Functions (Thin Delegation targets):
Line 201:   void AppInit_Task(void *argument);          — One-shot init (USB + car cmd), self-deletes
Line 202:   void AppCarControl_Task(void *argument);    — 10ms control loop (encoder→PID→motor)
Line 204:   #if APP_ENABLE_IMU → void AppIMUService_Task(void *argument); — 5ms IMU acquisition
Line 207:   #if APP_ENABLE_DISPLAY → void AppDisplay_Task(void *argument); — 100ms display update
Line 210:   #if APP_ENABLE_BROADCAST → void AppBroadcast_Task(void *argument); — BT broadcast
Line 213-214: #if APP_ENABLE_UART_FIFO → void AppUart3Rx_Task / AppUart4Rx_Task
Line 217:   #if APP_ENABLE_KEYBOARD → void AppKeyScan_Task(void *argument);
Line 220:   #if APP_ENABLE_MUSIC → void AppBuzzer_Task(void *argument);
Line 222:   void AppReserved_Task(void *argument);      — Placeholder for future features
Line 224-229: #if APP_ENABLE_SYSMON:
Line 225:   void AppSysMon_Task(void *argument);        — 1s snapshot, 200ms display refresh
Line 226:   void AppSysMon_UpdateSnapshot(void);        — Read heap + 4 task stack watermarks
Line 227:   size_t AppSysMon_BuildSystemSummary(buf, len); — Format heap/stack summary string
Line 228:   size_t AppSysMon_BuildRuntimeStats(buf, len); — vTaskGetRunTimeStats formatted output
Line 231-234: #if APP_ENABLE_CONSOLE:
Line 232:   void AppConsole_Task(void *argument);       — CLI input loop (USB CDC RX → cmd dispatch)
Line 233:   void AppConsole_Init(void);                 — Register FreeRTOS+CLI commands
Line 236-251: QSPI Flash Service Layer (all guarded by APP_ENABLE_QSPI):
Line 240:   bool        AppQSPI_Init(void);             — Init W25Q256 chip (detect + reset)
Line 241:   bool        AppQSPI_IsPresent(void);        — Check if flash chip is detected
Line 242:   bool        AppQSPI_Mount(void);            — Mount LittleFS on flash
Line 243:   void        AppQSPI_Umount(void);           — Unmount LittleFS
Line 244:   bool        AppQSPI_IsMounted(void);        — Query mount state
Line 245:   bool        AppQSPI_Format(void);           — Format flash with LittleFS
Line 246:   size_t      AppQSPI_ListDir(buf, len);      — List root directory
Line 247:   size_t      AppQSPI_ReadFile(buf, len, p, n); — Read file contents
Line 248:   const char* AppQSPI_GetName(void);          — "W25Q256 (32 MB)"
Line 249:   uint32_t    AppQSPI_ReadID(void);           — Return JEDEC manufacturer+device ID
Line 250:   size_t      AppQSPI_BuildInfo(buf, len);    — Build flash info summary string
Line 253:   #ifdef __cplusplus
Line 254:   }                                              — Close extern "C"
Line 255:   #endif
Line 257:   #endif /* __APP_H__ */                       — End include guard
```

### APP_ENABLE_* Switch Summary

| Switch | Default | Module | Code Files | RAM (Task Stack) |
|--------|---------|--------|-----------|-------------------|
| `APP_ENABLE_SYSMON` | 1 | System Monitor | `app_sysmon.c` | 2048 B |
| `APP_ENABLE_CONSOLE` | 1 | CLI Console | `app_console.c` + `FreeRTOS_CLI.c` | 2048 B |
| `APP_ENABLE_QSPI` | 1 | QSPI Flash + LittleFS | `app_qspi.c` + `w25q256.c` + `lfs_port.c` + `lfs.c` | N/A (no task) |
| `APP_ENABLE_DISPLAY` | 1 | OLED/LCD Display | `display_service.c` + `oled_spi_V0.2.c` | 512 B |
| `APP_ENABLE_KEYBOARD` | 1 | 4×4 Matrix Keyboard | `keyboard.c` | 512 B (Cube) + 256 B (FreeRTOS) |
| `APP_ENABLE_MUSIC` | 0 | Buzzer Music Player | (app_device.c) | 512 B |
| `APP_ENABLE_UART_FIFO` | 1 | UART3/UART4 Frame RX | `uart_fifo.c` | 1024 B × 2 |
| `APP_ENABLE_ROBOTIC_ARM` | 0 | Robotic Arm | (arm/ directories) | N/A |
| `APP_ENABLE_IMU` | 1 | IMU Sensor | `IMU.c` + `read_aux_data_mode.c` | 512 B |
| `APP_ENABLE_LEDBLINK` | 1 | LED Heartbeat | (freertos.c handler) | 512 B |
| `APP_ENABLE_BROADCAST` | 0 | BT UART4 Broadcast | (app_device.c) | 1024 B |

### All #include Transitive Dependencies

Through `app.h`, every CubeMX file gets access to:

| #include | Provides |
|----------|----------|
| `config.h` → 4 sub-configs | Platform dimensions, motor/control PID, RTOS timing |
| `delay.h` | `delay_init()`, `delay_ms()`, `delay_us()` |
| `sys_time.h` | `nowtime`, `SysTime_GetMs()`, `SysTime_GetUs()` |
| `pid.h` | `pid` struct, `PID_Cal_Inc()`, `PID_Cal_Pos()`, `PID_Clear()` |
| `PWM.h` | `Set_PWM_Duty()`, `dutyA/B/C/D`, `pwm_counter` |
| `Motor.h` | `motor` struct, 4 motor instances, `init_motor()`, wheel API |
| `car_attitude.h` | `_car_attitude` struct, attitude PID, yaw update |
| `car_control.h` | `_car_control` struct, control modes (GO_LINE/TO_POINT/SPIN) |
| `IMU.h` (if IMU enabled) | `IMU_init()`, `IMU_getYawPitchRoll()`, `imu_g_z` |
| `read_aux_data_mode.h` (if IMU) | `setup_imu()`, `bsp_IcmGetRawData()` |
| `uart_fifo.h` (if UART) | `UART_FRAME_MAX_LEN`, `uart_frame_rx_init()` |
| `keyboard.h` (if keyboard) | `Key_StartScanTask()`, `queue_keyHandle` |
| `oled_spi_V0.2.h` (if display) | `OLED_Init()`, `OLED_ShowString()`, `OLED_Clear()` |
| `lcd_st7789.h` (if display) | `SPI_LCD_Init()`, `LCD_DisplayString()`, `LCD_Clear()` |

---

## 2. config.h — Configuration Aggregator

**Path:** `Core/Inc/config.h` | **Lines:** 1-23

```
Line 1-11:  Doxygen comment explaining the 4 domain-specific configs:
            platform_config.h  — Wheel diameter, encoder lines, max speed
            motor_config.h     — 4-motor incremental PID limits + P/I/D
            control_config.h   — Position/angle/angular velocity PID limits + parameters
            rtos_config.h      — RTOS task periods, mode switches
Line 13:    #ifndef __CONFIG_H__                  — Include guard check
Line 14:    #define __CONFIG_H__                  — Include guard define
Line 16:    #include "platform_config.h"           — Wheel diameter, encoder pulses, frame dimensions
Line 17:    #include "motor_config.h"              — 4-motor PID limits + coefficients
Line 18:    #include "control_config.h"            — Position/angular PID limits + coefficients
Line 19:    #include "rtos_config.h"               — TASK_ITV_CAR, TASK_ITV_IMU, mode switches
Line 21:    #endif /* __CONFIG_H__ */              — End include guard
```

---

## 3. main.c — Entry Point

**Path:** `Core/Src/main.c` | **Lines:** 1-353 | **USER CODE sections only annotated**

### Line-by-Line Annotation

```
Line 1-18:  STM CubeMX generated copyright header block.
Line 20-29: System HAL includes:
Line 20:    #include "main.h"                       — CubeMX main HAL header
Line 21:    #include "FreeRTOS.h"                  — FreeRTOS kernel API
Line 22:    #include "cmsis_os2.h"                 — CMSIS-RTOS v2 wrapper
Line 23:    #include "quadspi.h"                   — QSPI peripheral HAL
Line 24:    #include "ramecc.h"                    — RAM ECC initialization
Line 25:    #include "spi.h"                       — SPI peripheral HAL (SPI2, SPI4)
Line 26:    #include "tim.h"                       — Timer peripheral HAL (TIM2-7, TIM13)
Line 27:    #include "usart.h"                     — USART/UART HAL (USART1/2/3, UART4/5)
Line 28:    #include "usb_device.h"                — USB Device HAL
Line 29:    #include "gpio.h"                      — GPIO HAL

Line 31-37: USER CODE BEGIN Includes:
Line 33:    #include "app.h"                       — Facade header: ALL user modules
Line 34-36: #if APP_ENABLE_DISPLAY:
Line 35:    #include "oledpicture_V0.2.h"           — OLED bitmap images (Genshin logo, etc.)

Line 39-42: USER CODE BEGIN PTD: (empty)           — Private typedef
Line 44-48: USER CODE BEGIN PD:                     — Private define
Line 46:    #define CPU_FREQ 480                   — CPU clock in MHz (for delay_init)
Line 47:    #define To_cm 2.54/102                 — Unit conversion (unused in current code)
Line 50-53: USER CODE BEGIN PM: (empty)             — Private macro
Line 55-62: USER CODE BEGIN PV:                     — Private variables
Line 58:    uint8_t cnt = 0;                       — Unused counter variable
Line 59:    float motion6[7];                      — IMU raw data [accel_x,accel_y,accel_z,
                                                          gyro_x,gyro_y,gyro_z,temp]
Line 60:    float ypr[3];                          — IMU attitude: yaw[0], pitch[1], roll[2]
Line 61:    int math_pl = 0;                       — Math placeholder (extern in app.h)
Line 64-70: USER CODE BEGIN PFP: (empty)            — Private function prototypes
Line 72-75: USER CODE BEGIN 0: (empty)              — Early private code

Line 81-184: int main(void):
Line 84-86:  USER CODE BEGIN 1: (empty — before MPU config)
Line 89:    MPU_Config();                          — Configure MPU Region 0 (full access, 4GB)
Line 94:    HAL_Init();                            — Reset peripherals, init Flash, init SysTick

Line 96-98:  USER CODE BEGIN Init:
Line 97:    init_motor();                          — Initialize all 4 motor structs:
                                                   Set GPIO direction pins, PID params, PWM channels

Line 101-103: SystemClock_Config(); → USER CODE BEGIN SysInit: (empty)

Line 108-122: CubeMX peripheral init sequence:
Line 108:   MX_GPIO_Init();                        — GPIO port clocks + pin configs
Line 109:   MX_TIM2_Init();                        — Timer 2: Encoder mode (Left Front motor)
Line 110:   MX_TIM3_Init();                        — Timer 3: Encoder mode (Left Rear motor)
Line 111:   MX_TIM4_Init();                        — Timer 4: Encoder mode (Right Front motor)
Line 112:   MX_TIM5_Init();                        — Timer 5: Encoder mode (Right Rear motor)
Line 113:   MX_TIM6_Init();                        — Timer 6: Software PWM base (+ update ISR)
Line 114:   MX_QUADSPI_Init();                     — QSPI: W25Q256 flash interface
Line 115:   MX_SPI4_Init();                        — SPI4: IMU (ICM-45686) interface
Line 116:   MX_TIM13_Init();                       — Timer 13: (spare timer)
Line 117:   MX_UART4_Init();                       — UART4: Bluetooth module / broadcast
Line 118:   MX_USART1_UART_Init();                 — USART1: (UART debug / control)
Line 119:   MX_SPI2_Init();                        — SPI2: OLED display interface
Line 120:   MX_USART2_UART_Init();                 — USART2: FT Servo serial (SCServo)
Line 121:   MX_USART3_UART_Init();                 — USART3: UART frame receive
Line 122:   MX_UART5_Init();                       — UART5: (spare)
Line 123:   MX_RAMECC_Init();                      — RAM ECC initialization

Line 124-164: USER CODE BEGIN 2: (Main initialization block)
Line 125:   #if APP_ENABLE_DISPLAY
Line 126:   OLED_Init();                           — Initialize SSD1306 OLED (SPI2)
Line 127:   #endif
Line 128:   #if APP_ENABLE_MUSIC
Line 129:   MusicPlayer_Init();                    — Initialize buzzer music player
Line 130:   #endif
Line 131:   HAL_Delay(1000);                       — 1-second power-up stabilization

Line 132-136: #if APP_ENABLE_DISPLAY:
              OLED_DrawBMP(9, 0, 119, 8, Genshin);  — Draw startup logo bitmap
              HAL_Delay(1000);                       — Show logo for 1 second
              OLED_Clear();                          — Clear display after startup

Line 137:   delay_init(CPU_FREQ);                  — Re-init DWT delay with 480MHz CPU freq
Line 138:   IMU_init();                            — Init ICM-45686 + quaternion = (1,0,0,0)
Line 139:   HAL_Delay(1000);                       — 1s IMU warmup

Line 140-149: Encoder and timer initialization:
Line 140:   HAL_TIM_Encoder_Start(&htim2, ALL);    — Start TIM2 encoder (Left Front)
Line 141:   HAL_TIM_Encoder_Start(&htim3, ALL);    — Start TIM3 encoder (Left Rear)
Line 142:   HAL_TIM_Encoder_Start(&htim4, ALL);    — Start TIM4 encoder (Right Front)
Line 143:   HAL_TIM_Encoder_Start(&htim5, ALL);    — Start TIM5 encoder (Right Rear)
Line 145:   HAL_TIM_Base_Start_IT(&htim2);         — Enable TIM2 update interrupt
Line 146:   HAL_TIM_Base_Start_IT(&htim3);         — Enable TIM3 update interrupt
Line 147:   HAL_TIM_Base_Start_IT(&htim4);         — Enable TIM4 update interrupt
Line 148:   HAL_TIM_Base_Start_IT(&htim5);         — Enable TIM5 update interrupt
Line 149:   HAL_TIM_Base_Start_IT(&htim6);         — Enable TIM6 interrupt (SW PWM + 1ms tick)

Line 151-154: Car control initialization:
Line 152:   init_Car_Attitude();                   — Set PID limits/coefficients for angular velocity PID
Line 153:   init_Car_Contorl();                    — Set PID limits/coefficients for position & spin PID
Line 154:   HAL_Delay(2000);                       — 2s stabilization delay
Line 155:   printf("init ok");                      — USB CDC debug output: init complete

Line 156-158: Flash initialization:
Line 157:   QSPI_W25Qxx_Init();                    — Init W25Q256: reset, read JEDEC ID, verify 0x4019
Line 158:   // QSPI_W25Qxx_Test();                — Flash R/W test (commented out)

Line 159-163: Robotic arm initialization (DISABLED):
Line 159:   // setup_roboticArm();                  — Robotic arm servo setup
Line 160:   HAL_Delay(1000);
Line 161:   // initArm();                           — Arm position initialization
Line 162:   HAL_Delay(1000);
Line 163:   // moveArm(150, 200, 50);               — Move arm to target position

Line 166-171: FreeRTOS start sequence:
Line 167:   osKernelInitialize();                  — CMSIS-OS2 wrapper: init FreeRTOS kernel objects
Line 168:   MX_FREERTOS_Init();                    — CubeMX: create tasks, queues, semaphores
Line 171:   osKernelStart();                       — Start FreeRTOS scheduler (never returns)

Line 176-183: while(1) { ... }                      — Fallback infinite loop (never reached)
```

### SystemClock_Config Function (lines 190-247)

This is a CubeMX-generated function — fully documented here because it defines the entire clock tree:

```
Line 190:   void SystemClock_Config(void):
Line 192-193: RCC_OscInitTypeDef + RCC_ClkInitTypeDef zero-initialize
Line 197:     HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY) — Enable LDO supply

Line 201:     __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0)
              — Voltage Scale 0: max CPU frequency (480 MHz), VOS0 range

Line 203:     while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
              — Wait for voltage regulator ready (Scale 0 ramp-up)

Line 208-220: PLL1 oscillator config:
Line 208:       OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE
Line 209:       HSEState = RCC_HSE_ON             — Enable 25 MHz external crystal
Line 210:       HSI48State = RCC_HSI48_ON         — Enable 48 MHz RC (for USB OTG FS)
Line 211:       PLL.PLLState = RCC_PLL_ON         — Enable main PLL
Line 212:       PLL.PLLSource = RCC_PLLSOURCE_HSE — Clock source: 25 MHz HSE
Line 213:       PLL.PLLM = 5                      — VCO input = 25MHz / 5 = 5 MHz
Line 214:       PLL.PLLN = 192                    — VCO = 5MHz × 192 = 960 MHz
Line 215:       PLL.PLLP = 2                      — sys_ck (PLL1_p) = 960/2 = 480 MHz
Line 216:       PLL.PLLQ = 2                      — qspi_ck = 960/2 = 480 MHz
Line 217:       PLL.PLLR = 2                      — pll1_r = 480 MHz (backup/per_ck)
Line 218:       PLLRGE = RCC_PLL1VCIRANGE_2       — VCO input 4-8 MHz range
Line 219:       PLLVCOSEL = RCC_PLL1VCOWIDE       — Wide VCO range (192-960 MHz)
Line 221-224: if HAL_RCC_OscConfig fails → Error_Handler()

Line 228-237: Bus clock dividers:
Line 229:       SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK
Line 230:       SYSCLKDivider = RCC_SYSCLK_DIV1   — SYSCLK = 480 MHz
Line 231:       AHBCLKDivider = RCC_HCLK_DIV2     — HCLK = 240 MHz (AXI/AHB)
Line 232:       APB3CLKDivider = RCC_APB3_DIV2    — APB3 = 120 MHz
Line 233:       APB1CLKDivider = RCC_APB1_DIV2    — APB1 = 120 MHz
Line 234:       APB2CLKDivider = RCC_APB2_DIV2    — APB2 = 120 MHz
Line 235:       APB4CLKDivider = RCC_APB4_DIV2    — APB4 = 120 MHz

Line 239-242: if HAL_RCC_ClockConfig(&cfg, FLASH_LATENCY_4) fails → Error_Handler()
              FLASH_LATENCY_4: 4 CPU wait states (required for 480 MHz VOS0)

Line 244:     HAL_RCC_EnableCSS()                 — Enable Clock Security System
              If HSE fails → HSI48 takes over, NMI triggered

Clock Tree Summary:
  HSE (25 MHz)
    └─ /PLLM(5) → 5 MHz → ×PLLN(192) → 960 MHz VCO
         ├─ /PLLP(2) → 480 MHz SYSCLK
         │    └─ /HPRE(2) → 240 MHz HCLK → CPU + AXI + SysTick
         ├─ /PLLQ(2) → 480 MHz qspi_ck (QSPI peripheral)
         └─ /PLLR(2) → 480 MHz per_ck
```

### USER CODE BEGIN 4 — FT Servo Support (lines 249-271)

```
Line 250-254: void ftUart_Send(uint8_t *nDat, int nLen):
              HAL_UART_Transmit(&huart2, nDat, nLen, 100) — Send raw bytes via USART2
              Used by SCServo library for FT servo command transmission.

Line 257-264: int ftUart_Read(uint8_t *nDat, int nLen):
              HAL_UART_Receive(&huart2, nDat, nLen, 100) — Blocking read from USART2
              Returns nLen on success, 0 on timeout/failure.
              Used by SCServo library for FT servo response reception.

Line 268-270: void ftBus_Delay(void):
              HAL_Delay(1) — 1ms minimum bus switching delay (>10us required)
              Used by SCServo library to prevent bus contention after direction change.
```

### MPU_Config (lines 275-300)

```
Line 275:   void MPU_Config(void):
Line 280:     HAL_MPU_Disable()                   — Disable MPU before reconfiguring
Line 285:       Enable = MPU_REGION_ENABLE        — Enable this MPU region
Line 286:       Number = MPU_REGION_NUMBER0       — Region 0 (highest priority)
Line 287:       BaseAddress = 0x0                 — Covers entire 4 GB address space
Line 288:       Size = MPU_REGION_SIZE_4GB        — 4 GB region size
Line 289:       SubRegionDisable = 0x87           — Disable sub-regions 0,1,2,7
                                                (bits 0111_1000 → disable 0,1,2,7)
Line 290:       TypeExtField = MPU_TEX_LEVEL0     — C/B bits come from MPU_RASR
Line 291:       AccessPermission = MPU_REGION_FULL_ACCESS — R/W in both priviledged and user
Line 292:       DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE — Disable instruction fetch
Line 293:       IsShareable = MPU_ACCESS_SHAREABLE — Memory is shareable
Line 294:       IsCacheable = MPU_ACCESS_NOT_CACHEABLE — No data caching
Line 295:       IsBufferable = MPU_ACCESS_NOT_BUFFERABLE — No write buffering
Line 296:     HAL_MPU_ConfigRegion(&MPU_InitStruct) — Apply region config
Line 298:     HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT) — Enable MPU for priviledged threads
```

### HAL_TIM_PeriodElapsedCallback (lines 310-322)

```
Line 310-322: void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim):
Line 315:     if (htim->Instance == TIM7)          — TIM7 serves as HAL timebase
Line 317:       HAL_IncTick()                      — Increment global uwTick (HAL tick counter)
```

### Error_Handler + assert_failed (lines 328-353)

```
Line 328-337: void Error_Handler(void):
              __disable_irq()                      — Disable all interrupts
              while(1) {}                          — Infinite loop (fault state)

Line 346-353: void assert_failed(uint8_t *file, uint32_t line):
              Empty stub — user can add printf or logging here.
              Only compiled when USE_FULL_ASSERT is defined.
```

---

## 4. freertos.c — RTOS Configuration

**Path:** `Core/Src/freertos.c` | **Lines:** 1-599

### Line-by-Line Annotation

```
Line 1-18:  STM CubeMX copyright header block
Line 20-25: System FreeRTOS includes:
Line 21:    #include "FreeRTOS.h"                  — FreeRTOS kernel API
Line 22:    #include "task.h"                      — Task management API
Line 23:    #include "main.h"                      — CubeMX main header
Line 24:    #include "FreeRTOS.h"                  — (duplicate include)
Line 25:    #include "cmsis_os2.h"                 — CMSIS-RTOS v2 wrapper

Line 27-30: USER CODE BEGIN Includes:
Line 29:    #include "app.h"                       — Facade header: all user modules

Line 33-36: Private typedef:
Line 33:    typedef StaticTask_t osStaticThreadDef_t; — Alias for static task TCB type

Line 38-46: USER CODE BEGIN PD / PM: (empty)

Line 48-59: USER CODE BEGIN Variables:
Line 50:    extern float motion6[7];               — IMU raw data (defined main.c:59)
Line 51:    extern float ypr[3];                   — IMU attitude angles (defined main.c:60)
Line 52:    extern int math_pl;                    — Math placeholder (defined main.c:61)
Line 53:    QueueHandle_t uart3_frame_queue;        — UART3 frame receive queue (5 slots × 64B)
Line 54:    QueueHandle_t uart4_frame_queue;        — UART4 frame receive queue (5 slots × 64B)
Line 55:    TaskHandle_t xMotionTaskHandle = NULL;  — Motion task handle (unused, reserved)
Line 56:    StreamBufferHandle_t console_rx_stream = NULL; — USB CDC RX stream buffer (256B)
```

### Static Task Definitions (lines 60-203)

Each task definition follows this pattern:
- `osThreadId_t XxxHandle` — CMSIS thread ID variable
- `uint32_t XxxBuffer[N]` — Stack memory (N × 4 bytes)
- `osStaticThreadDef_t XxxControlBlock` — Static TCB storage
- `const osThreadAttr_t Xxx_attributes` — Task attributes: name, TCB ptr, stack ptr, stack size, priority

```
Line 60-71:  LEDBlink task:
Line 61:    osThreadId_t LEDBlinkHandle;           — Task handle
Line 62:    uint32_t LEDBlinkBuffer[128];          — Stack: 128 words = 512 bytes
Line 63:    osStaticThreadDef_t LEDBlinkControlBlock; — Static TCB
Line 64-71: LEDBlink_attributes:
Line 65:      .name = "LEDBlink"                   — Task name (debug identification)
Line 66:      .cb_mem = &LEDBlinkControlBlock      — Pointer to static TCB
Line 67:      .cb_size = sizeof(LEDBlinkControlBlock)
Line 68:      .stack_mem = &LEDBlinkBuffer[0]      — Pointer to stack memory
Line 69:      .stack_size = sizeof(LEDBlinkBuffer) — 512 bytes
Line 70:      .priority = osPriorityAboveNormal    — Higher than normal (USB init needs timing)

Line 72-83:  Broadcast task:
             Stack: 256 × 4 = 1024 bytes | Priority: osPriorityNormal1

Line 84-95:  Uart4Rx task:
             Stack: 256 × 4 = 1024 bytes | Priority: osPriorityNormal7 (high)

Line 96-107: IMUService task:
             Stack: 128 × 4 = 512 bytes | Priority: osPriorityNormal3 (medium)

Line 108-119: CarControl task:
              Stack: 128 × 4 = 512 bytes | Priority: osPriorityNormal5 (medium-high)

Line 120-131: KeyScan task:
              Stack: 128 × 4 = 512 bytes | Priority: osPriorityNormal6

Line 132-143: Buzzer task:
              Stack: 128 × 4 = 512 bytes | Priority: osPriorityNormal1 (low)

Line 144-155: Display task:
              Stack: 128 × 4 = 512 bytes | Priority: osPriorityNormal1

Line 156-167: Reserved task:
              Stack: 1024 × 4 = 4096 bytes (largest — room for future extensions)
              Priority: osPriorityNormal1

Line 168-179: Uart3Rx task:
              Stack: 256 × 4 = 1024 bytes | Priority: osPriorityNormal7 (high)

Line 180-191: SysMon task:
              Stack: 512 × 4 = 2048 bytes (needs space for snprintf buffer)
              Priority: osPriorityLow (lowest priority)

Line 192-203: Console task:
              Stack: 512 × 4 = 2048 bytes | Priority: osPriorityLow
```

### Kernel Object Definitions (lines 204-218)

```
Line 204-208: osMessageQueueId_t queue_keyHandle:
              Created with osMessageQueueNew(1, sizeof(char), &attrs)
              — Single-slot character queue for keyboard key events

Line 209-213: osSemaphoreId_t semphr_buzzer_triggerHandle:
              Created with osSemaphoreNew(1, 1, &attrs)
              — Binary semaphore (max=1, initial=1) for buzzer trigger

Line 214-218: osSemaphoreId_t semphr_uart_receiveHandle:
              Created with osSemaphoreNew(16, 0, &attrs)
              — Counting semaphore (max=16, initial=0) for UART receive notifications
```

### MX_FREERTOS_Init Function (lines 290-379)

```
Line 290: void MX_FREERTOS_Init(void):
Line 291-293: USER CODE BEGIN Init: (empty)
Line 295-297: USER CODE BEGIN RTOS_MUTEX: (empty)
Line 301:   semphr_buzzer_triggerHandle = osSemaphoreNew(1, 1, &attrs) — Create buzzer semaphore
Line 304:   semphr_uart_receiveHandle = osSemaphoreNew(16, 0, &attrs) — Create UART semaphore
Line 306-308: USER CODE BEGIN RTOS_SEMAPHORES: (empty)
Line 310-312: USER CODE BEGIN RTOS_TIMERS: (empty)

Line 316:   queue_keyHandle = osMessageQueueNew(1, sizeof(char), &attrs) — Key event queue

Line 318-326: USER CODE BEGIN RTOS_QUEUES:
Line 320-321: #if APP_ENABLE_UART_FIFO:
              uart3_frame_queue = xQueueCreate(5, UART_FRAME_MAX_LEN) — 5 frames × 64 bytes
              uart4_frame_queue = xQueueCreate(5, UART_FRAME_MAX_LEN) — 5 frames × 64 bytes
Line 324:    #if APP_ENABLE_CONSOLE:
              console_rx_stream = xStreamBufferCreate(256, 1) — 256 bytes, trigger=1

Line 328-363: Task creation via osThreadNew (12 tasks):
Line 330:     LEDBlinkHandle = osThreadNew(LedBlink_Handler, NULL, &attrs)
Line 333:     BroadcastHandle = osThreadNew(Broadcast_Handler, NULL, &attrs)
Line 336:     Uart4RxHandle = osThreadNew(Uart4Rx_Handler, NULL, &attrs)
Line 339:     IMUServiceHandle = osThreadNew(IMUService_Handler, NULL, &attrs)
Line 342:     CarControlHandle = osThreadNew(CarControl_Handler, NULL, &attrs)
Line 345:     KeyScanHandle = osThreadNew(KeyScan_Handler, NULL, &attrs)
Line 348:     BuzzerHandle = osThreadNew(Buzzer_Handler, NULL, &attrs)
Line 351:     DisplayHandle = osThreadNew(Display_Handler, NULL, &attrs)
Line 354:     ReservedHandle = osThreadNew(Reserved_Handler, NULL, &attrs)
Line 357:     Uart3RxHandle = osThreadNew(Uart3Rx_Handler, NULL, &attrs)
Line 360:     SysMonHandle = osThreadNew(SysMon_Handler, NULL, &attrs)
Line 363:     ConsoleHandle = osThreadNew(Console_Handler, NULL, &attrs)

Line 365-373: USER CODE BEGIN RTOS_THREADS:
Line 367:     #if APP_ENABLE_KEYBOARD → Key_StartScanTask()
              — Creates FreeRTOS task "KeyScan" (alternative to Cube task)
Line 369:     xTaskCreate(AppInit_Task, "AppInit", 512, NULL, osPriorityNormal5, NULL)
              — Dynamic task: AppInit (512 words = 2048 bytes stack)
              — Runs once (USB init + car command), then self-deletes
Line 370-372: #if APP_ENABLE_CONSOLE → AppConsole_Init()
              — Register FreeRTOS+CLI commands before console task starts
Line 375-377: USER CODE BEGIN RTOS_EVENTS: (empty)
```

### FreeRTOS Hooks (lines 246-283)

```
Line 246-257: void vApplicationIdleHook(void):
              Empty — called on each idle task iteration
              MUST never block, call vTaskDelay, or use queues with timeout.

Line 261-266: __weak void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName):
              __weak stub — real implementation in app_rtos_hooks.c overrides
              Called when configCHECK_FOR_STACK_OVERFLOW detects overflow.

Line 270-282: __weak void vApplicationMallocFailedHook(void):
              __weak stub — real implementation in app_rtos_hooks.c overrides
              Called when pvPortMalloc fails (heap_4 out of memory).
```

### All 12 Handler Function Bodies (lines 381-599)

```
Line 381-403: LedBlink_Handler(void *argument):
Line 391:   MX_USB_DEVICE_Init()                   — USB CDC device initialization
             (MUST be done after scheduler starts — OS dependency)
Line 393-401: USER CODE BEGIN LedBlink_Handler:
Line 394:   #if APP_ENABLE_LEDBLINK:
Line 395:     for(;;) {
Line 396:       HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin); — Toggle LED state
Line 397:       osDelay(1000);                      — Delay 1 second → 0.5 Hz blink rate
Line 398:     }
Line 399-400: #else: vTaskDelete(NULL);             — Self-delete disabled task


Line 405-421: Broadcast_Handler:
Line 415:   #if APP_ENABLE_BROADCAST:
              AppBroadcast_Task(argument)            — Delegate to app_device.c
Line 418:   #else: vTaskDelete(NULL);                — Self-delete disabled task

Line 423-439: Uart4Rx_Handler:
Line 433:   #if APP_ENABLE_UART_FIFO:
              AppUart4Rx_Task(argument)              — Delegate to app_device.c
Line 436:   #else: vTaskDelete(NULL);

Line 441-457: IMUService_Handler:
Line 451:   #if APP_ENABLE_IMU:
              AppIMUService_Task(argument)           — Delegate to app_sensor.c
Line 454:   #else: vTaskDelete(NULL);

Line 459-471: CarControl_Handler:
Line 469:   AppCarControl_Task(argument)               — Always enabled, delegate to app_control.c

Line 473-489: KeyScan_Handler:
Line 483:   #if APP_ENABLE_KEYBOARD:
              AppKeyScan_Task(argument)              — Delegate to app_device.c
Line 486:   #else: vTaskDelete(NULL);

Line 491-507: Buzzer_Handler:
Line 501:   #if APP_ENABLE_MUSIC:
              AppBuzzer_Task(argument)               — Delegate to app_device.c
Line 504:   #else: vTaskDelete(NULL);

Line 509-525: Display_Handler:
Line 519:   #if APP_ENABLE_DISPLAY:
              AppDisplay_Task(argument)              — Delegate to app_device.c
Line 522:   #else: vTaskDelete(NULL);

Line 527-539: Reserved_Handler:
Line 537:   AppReserved_Task(argument)                 — Delegate to app_device.c

Line 541-557: Uart3Rx_Handler:
Line 551:   #if APP_ENABLE_UART_FIFO:
              AppUart3Rx_Task(argument)              — Delegate to app_device.c
Line 554:   #else: vTaskDelete(NULL);

Line 559-575: SysMon_Handler:
Line 569:   #if APP_ENABLE_SYSMON:
              AppSysMon_Task(argument)               — Delegate to app_sysmon.c
Line 572:   #else: vTaskDelete(NULL);

Line 577-593: Console_Handler:
Line 587:   #if APP_ENABLE_CONSOLE:
              AppConsole_Task(argument)              — Delegate to app_console.c
Line 590:   #else: vTaskDelete(NULL);

Line 595-599: USER CODE BEGIN Application: (empty)
```

---

## 5. FreeRTOSConfig.h — Kernel Configuration

**Path:** `Core/Inc/FreeRTOSConfig.h` | **Lines:** 1-185

### Line-by-Line Annotation

```
Line 1-29:  Amazon/ST copyright header for FreeRTOS V10.3.1
Line 30:    #ifndef FREERTOS_CONFIG_H               — Include guard
Line 31:    #define FREERTOS_CONFIG_H               — Guard define

Line 45-47: USER CODE BEGIN Includes: (empty)

Line 50-56: Port includes:
Line 51:    #include <stdint.h>                       — Standard integer types
Line 52:    extern uint32_t SystemCoreClock;         — Defined by CMSIS/SystemInit
Line 54-56: #ifndef CMSIS_device_header → #define "stm32h7xx.h"

--- KERNEL FEATURE FLAGS (lines 58-99) ---

Line 58:    #define configENABLE_FPU                      1
              — Enable hardware FPU context saving for each task
Line 59:    #define configENABLE_MPU                      0
              — MPU managed by application (main.c:MPU_Config), not FreeRTOS

Line 61:    #define configUSE_PREEMPTION                  1
              — Preemptive scheduler (tasks can preempt each other)
Line 62:    #define configSUPPORT_STATIC_ALLOCATION       1
              — Enable static task/queue/semaphore creation (no heap needed)
Line 63:    #define configSUPPORT_DYNAMIC_ALLOCATION      1
              — Enable dynamic allocation too (AppInit task uses it)

Line 64:    #define configUSE_IDLE_HOOK                   1
              — Call vApplicationIdleHook() on each idle task iteration
Line 65:    #define configUSE_TICK_HOOK                   0
              — Do NOT call tick hook (saves cycles)

Line 66:    #define configCPU_CLOCK_HZ (SystemCoreClock)
              — Initial value = 480 MHz → overridden in USER CODE to 240 MHz
Line 67:    #define configTICK_RATE_HZ ((TickType_t)1000)
              — System tick rate: 1000 Hz → 1 ms per tick
Line 68:    #define configMAX_PRIORITIES (56)
              — 0..55 priority levels (55 = highest for timer task)
Line 69:    #define configUSE_SB_COMPLETED_CALLBACK (0)
              — Disable stream buffer completed callback
Line 70:    #define configUSE_MINI_LIST_ITEM (1)
              — Use compact list item struct (saves RAM)
Line 71:    #define configMINIMAL_STACK_SIZE ((uint16_t)128)
              — Minimum stack: 128 words = 512 bytes
Line 72:    #define configTOTAL_HEAP_SIZE ((size_t)81920)
              — Total heap: 81920 bytes = 80 KB (heap_4.c)
Line 73:    #define configMAX_TASK_NAME_LEN (32)
              — Max 32 characters per task name (for debug)
Line 74:    #define configHEAP_CLEAR_MEMORY_ON_FREE       0
              — Don't zero freed memory (faster, less secure)
Line 75:    #define configUSE_TRACE_FACILITY              1
              — Enable trace macros for debugging/visualization
Line 76:    #define configUSE_STATS_FORMATTING_FUNCTIONS  1
              — Enable vTaskList() and vTaskGetRunTimeStats() formatting
Line 77:    #define configUSE_16_BIT_TICKS                0
              — Use 32-bit tick counter (no overflow for ~49 days at 1kHz)
Line 78:    #define configUSE_MUTEXES                     1
              — Enable mutex semaphores (with priority inheritance)
Line 79:    #define configQUEUE_REGISTRY_SIZE             10
              — Queue registry capacity for debug
Line 80:    #define configCHECK_FOR_STACK_OVERFLOW        2
              — Method 2: check stack canary on task switch
Line 81:    #define configUSE_RECURSIVE_MUTEXES           1
              — Enable recursive mutexes
Line 82:    #define configUSE_MALLOC_FAILED_HOOK          1
              — Call vApplicationMallocFailedHook() on OOM
Line 83:    #define configUSE_COUNTING_SEMAPHORES         1
              — Enable counting semaphores
Line 84:    #define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
              — Generic task selection (portable, not HW-optimized)

Line 85-89: USER CODE BEGIN MESSAGE_BUFFER_LENGTH_TYPE:
Line 88:    #define configMESSAGE_BUFFER_LENGTH_TYPE size_t
              — Stream buffer length uses size_t

--- CO-ROUTINES (lines 92-93) ---
Line 92:    #define configUSE_CO_ROUTINES 0               — Disabled
Line 93:    #define configMAX_CO_ROUTINE_PRIORITIES (2)

--- SOFTWARE TIMERS (lines 96-99) ---
Line 96:    #define configUSE_TIMERS 1                    — Enable software timers
Line 97:    #define configTIMER_TASK_PRIORITY (55)        — Highest priority (timing critical)
Line 98:    #define configTIMER_QUEUE_LENGTH 10           — Timer command queue depth
Line 99:    #define configTIMER_TASK_STACK_DEPTH 256      — 256 words = 1024 bytes

--- CMSIS-RTOS V2 FLAGS (lines 102-122) ---
Line 102:   #define configUSE_OS2_THREAD_SUSPEND_RESUME   1
Line 103:   #define configUSE_OS2_THREAD_ENUMERATE        1
Line 104:   #define configUSE_OS2_EVENTFLAGS_FROM_ISR     1
Line 105:   #define configUSE_OS2_THREAD_FLAGS            1
Line 106:   #define configUSE_OS2_TIMER                   1
Line 107:   #define configUSE_OS2_MUTEX                   1

Line 110-122: Include individual FreeRTOS API functions:
Line 110:   INCLUDE_vTaskPrioritySet      1   — vTaskPrioritySet()
Line 111:   INCLUDE_uxTaskPriorityGet     1   — uxTaskPriorityGet()
Line 112:   INCLUDE_vTaskDelete           1   — vTaskDelete() (used by AppInit)
Line 113:   INCLUDE_vTaskCleanUpResources 1   — vTaskCleanUpResources()
Line 114:   INCLUDE_vTaskSuspend          1   — vTaskSuspend() / vTaskResume()
Line 115:   INCLUDE_xTaskDelayUntil       1   — xTaskDelayUntil() (used by CarControl, IMU)
Line 116:   INCLUDE_vTaskDelay            1   — vTaskDelay()
Line 117:   INCLUDE_xTaskGetSchedulerState 1   — xTaskGetSchedulerState()
Line 118:   INCLUDE_xTimerPendFunctionCall 1   — xTimerPendFunctionCall()
Line 119:   INCLUDE_xQueueGetMutexHolder  1   — xQueueGetMutexHolder()
Line 120:   INCLUDE_uxTaskGetStackHighWaterMark 1 — used by AppSysMon
Line 121:   INCLUDE_xTaskGetCurrentTaskHandle   1
Line 122:   INCLUDE_eTaskGetState               1

Line 128:   #define USE_FreeRTOS_HEAP_4
              — Use heap_4.c: coalesces adjacent free blocks, best general-purpose

Line 131-136: Cortex-M priority bit definitions:
              configPRIO_BITS = __NVIC_PRIO_BITS (4 for Cortex-M7)

--- INTERRUPT PRIORITY CONFIGURATION (lines 141-154) ---
Line 141:   #define configLIBRARY_LOWEST_INTERRUPT_PRIORITY    15
              — Lowest priority in STM32 NVIC library scale (0-15)
Line 147:   #define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
              — Highest priority that can call ISR-safe FreeRTOS API
              — Priority 0-4 CANNOT call FreeRTOS ISR functions
Line 151:   configKERNEL_INTERRUPT_PRIORITY = 15 << 4 = 240
              — Kernel interrupt priority (lowest, in shifted form)
Line 154:   configMAX_SYSCALL_INTERRUPT_PRIORITY = 5 << 4 = 80
              — Max ISR-syscall priority (in shifted form)

Line 158-161: USER CODE BEGIN 1:
Line 159:   #define configASSERT(x) if ((x)==0) { taskDISABLE_INTERRUPTS(); for(;;); }
              — Assert macro: disable IRQs and halt on assertion failure
Line 160:   #define configGENERATE_RUN_TIME_STATS            1
              — Enable runtime statistics (requires DWT->CYCCNT from delay_init)

Line 165-169: Cortex-M exception handler name mapping:
Line 166:   #define vPortSVCHandler    SVC_Handler
Line 167:   #define xPortPendSVHandler PendSV_Handler
Line 169:   #define USE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION 0
              — Use standard FreeRTOS SysTick implementation

Line 171-183: USER CODE BEGIN Defines:
Line 172:   #define configCOMMAND_INT_MAX_OUTPUT_SIZE          768
              — FreeRTOS+CLI max output buffer per command
Line 173:   #define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS    configureTimerForRunTimeStats
Line 174:   #define portGET_RUN_TIME_COUNTER_VALUE             getRunTimeCounterValue
              — Map runtime stats timer functions to DWT CYCCNT
Line 175-182: CRITICAL CPU CLOCK FIX:
              STM32H750 SysTick clock = HCLK (NOT SYSCLK!)
              HCLK = SYSCLK / D1CPRE / HPRE = 480 / 1 / 2 = 240 MHz
Line 181:   #undef  configCPU_CLOCK_HZ                 — Remove CubeMX default (480 MHz)
Line 182:   #define configCPU_CLOCK_HZ (SystemCoreClock / 2U) — Set to 240 MHz
              Without this override, SysTick runs at half rate → 2ms ticks

Line 185:   #endif /* FREERTOS_CONFIG_H */               — End include guard
```

### Critical FreeRTOSConfig Values Summary

| Parameter | Value | Impact |
|-----------|-------|--------|
| `configTICK_RATE_HZ` | 1000 | 1 ms scheduler tick |
| `configCPU_CLOCK_HZ` | 240 MHz | MUST match HCLK (SYSCLK/2) |
| `configTOTAL_HEAP_SIZE` | 81920 (80 KB) | Dynamic allocation pool |
| `configMAX_PRIORITIES` | 56 | Priority inversion headroom |
| `configTIMER_TASK_PRIORITY` | 55 | Highest: software timers are time-critical |
| `configCHECK_FOR_STACK_OVERFLOW` | 2 | Canary check on context switch |
| `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` | 5 | ISRs with prio ≥5 may call FreeRTOS API |

---

## 6. Configuration Files

### 6.1 platform_config.h — Platform Constants

**Path:** `Core/Inc/platform_config.h` | **Lines:** 1-46

```
Line 1-6:   File header and include guard
Line 10:    #define RAD_TO_DEGREE   57.29578018F    — Radians → degrees multiplier (180/π)
Line 11:    #define DEGREE_TO_RAD   0.01745329238F  — Degrees → radians multiplier (π/180)

Line 13:    #define USE_4_MOTOR     1                — 4-wheel drive mode
              1 = all 4 motors controlled, 0 = front 2 only

Line 16:    #define USE_4_TIMES_ENCODER 1            — Quadrature 4× decoding
              Each encoder pulse generates 4 counts (rising/falling on both channels)

Line 19:    #define ENC_EVERY_CIRCLE        1061.268F — Encoder counts per wheel revolution
              Derived from physical encoder PPR × 4 × gear ratio

Line 22:    #define WHEEL_DIR               47F       — Wheel diameter in mm

Line 25:    #define WHEEL_PERIMETER         148.722996F — Wheel circumference = π × 47 ≈ 147.65 mm
              (adjusted value slightly different from π×47 due to measured calibration)

Line 28:    #define FRAME_W_HALF            66.25F    — Half track width (left-right wheel distance/2)
              Measured via IMU calibration

Line 31:    #define FRAME_L_HALF            55.85F    — Half wheelbase (front-rear wheel distance/2)

Line 34:    #define V_REAL_TO_ENC           (ENC_EVERY_CIRCLE / WHEEL_PERIMETER)
              — Encoder speed → real speed conversion: ~1061.268/148.723 ≈ 7.136 enc/mm
              Meaning: each mm of travel = ~7.136 encoder counts

Line 37:    #define MAX_V_ENC               (2 * ENC_EVERY_CIRCLE)
              — Max encoder speed: ~2122.5 enc/s (2 rev/s)

Line 40:    #define MAX_V_REAL              (MAX_V_ENC / V_REAL_TO_ENC)
              — Max real speed: ~2122.5 / 7.136 ≈ 297.4 mm/s

Line 43:    #define MAX_V_ANGLE             60.0F     — Max angular velocity: 60 degrees/second

Line 46:    #endif /* __PLATFORM_CONFIG_H__ */
```

### 6.2 motor_config.h — Motor PID Parameters

**Path:** `Core/Inc/motor_config.h` | **Lines:** 1-52

```
Line 1-9:   File header and include guard, #include "platform_config.h"
Line 12:    #define ZOOM_PID_TO_DUTY        0.001F    — PID output to duty ratio scaling
              Motor PID output × 0.001 = PWM duty cycle (0.0 to ~0.999)

Line 15:    #define MAX_MOTOR_DUTY          0.999F    — Max PWM duty (99.9%, ~0.1% dead time)

Line 18-21: Incremental PID output LIMITS (per motor):
Line 18:    #define LIMIT_INC_LF   0.5F × 0.999 / 0.001 ≈ 499.5  — Left Front incremental limit
Line 19:    #define LIMIT_INC_LR   0.5F × 0.999 / 0.001 ≈ 499.5  — Left Rear
Line 20:    #define LIMIT_INC_RF   0.5F × 0.999 / 0.001 ≈ 499.5  — Right Front
Line 21:    #define LIMIT_INC_RR   0.5F × 0.999 / 0.001 ≈ 499.5  — Right Rear

Line 24-27: Positional PID output LIMITS (per motor):
Line 24:    #define LIMIT_POS_LF   (MAX_MOTOR_DUTY × 1000) ≈ 999.0
Line 25-27: LIMIT_POS_LR/RF/RR same value

Line 30-34: Integral error LIMITS:
Line 30:    #define LIMIT_ITGR_MAX  1000000000.0F   — 1e9 (virtually unlimited)
Line 31-34: LIMIT_ITGR_LF/LR/RF/RR = LIMIT_ITGR_MAX

Line 37-50: PID Coefficients (Incremental PID for speed control):
Line 37-40: P (Proportional) gains: ALL 0.02F
              P_LF=0.02, P_LR=0.02, P_RF=0.02, P_RR=0.02
Line 42-45: I (Integral) gains: ALL 0.01F
              I_LF=0.01, I_LR=0.01, I_RF=0.01, I_RR=0.01
Line 47-50: D (Derivative) gains: ALL 0.015F
              D_LF=0.015, D_LR=0.015, D_RF=0.015, D_RR=0.015

Line 52:    #endif /* __MOTOR_CONFIG_H__ */
```

### 6.3 control_config.h — Control PID Parameters

**Path:** `Core/Inc/control_config.h` | **Lines:** 1-45

```
Line 1-9:   File header and include guard, #include "platform_config.h"

Line 12-16: Incremental PID limits:
Line 12:    #define LIMIT_INC_POS     (MAX_V_REAL × 0.5F) ≈ 148.7 mm/s
              — Position loop incremental output limit
Line 13:    #define LIMIT_INC_SPIN    MAX_V_ANGLE = 60.0 deg/s
              — Spin loop incremental limit
Line 14:    #define LIMIT_INC_V_ANGLE (0.5F × 60.0 × 66.25 × 0.01745) ≈ 34.69 mm/s
              — Angular velocity loop incremental limit (converted to mm/s equivalent)
Line 15-16: #define LIMIT_INC_VITUAL_X/Y  0.0F  — Virtual axis limits (disabled)

Line 18-23: Positional PID output limits:
Line 19:    #define LIMIT_POS_POS     MAX_V_REAL ≈ 297.4 mm/s
              — Position loop max output
Line 20:    #define LIMIT_POS_SPIN    MAX_V_ANGLE = 60.0 deg/s
              — Spin loop max output
Line 21:    #define LIMIT_POS_V_ANGLE (MAX_V_ANGLE × FRAME_W_HALF × DEGREE_TO_RAD) ≈ 69.38 mm/s
              — Angular velocity loop max output
Line 22-23: LIMIT_POS_VITUAL_X/Y = 100.0F

Line 25-30: Integral limits (all = LIMIT_ITGR_MAX = 1e9, from motor_config.h)

Line 33-43: PID Coefficients (Positional PID for motion control):
Line 33-35: P gains:
Line 33:    #define P_POS            2.0F   — Position loop proportional gain (distance PID)
Line 34:    #define P_SPIN           2.0F   — Spin loop proportional gain (angle PID)
Line 35:    #define P_V_ANGLE        1.0F   — Angular velocity loop proportional gain
Line 37-39: I gains:
Line 37:    #define I_POS            0.01F  — Position loop integral
Line 38:    #define I_SPIN           0.01F  — Spin loop integral
Line 39:    #define I_V_ANGLE        0.63F  — Angular velocity loop integral
Line 41-43: D gains:
Line 41:    #define D_POS            0.1F   — Position loop derivative
Line 42:    #define D_SPIN           0.5F   — Spin loop derivative
Line 43:    #define D_V_ANGLE        0.01F  — Angular velocity loop derivative

Line 45:    #endif /* __CONTROL_CONFIG_H__ */
```

### 6.4 rtos_config.h — RTOS Task Timing

**Path:** `Core/Inc/rtos_config.h` | **Lines:** 1-23

```
Line 1-7:   File header and include guard

Line 10:    #define TASK_ITV_CAR        10     — Car control loop period: 10 ms (100 Hz)
Line 11:    #define TASK_ITV_IMU        5      — IMU data acquisition period: 5 ms (200 Hz)
Line 12:    #define TASK_ITV_UPLOAD     50     — Data upload period: 50 ms (20 Hz)

Line 15:    #define USE_CAR_CONTROL     1      — Enable position PID closed-loop
              1 = use position/angle PID (car_control.c)
              0 = direct attitude control (bypass position loop)

Line 18:    #define V_ANGLE_PID         1      — Enable angular velocity PID
              1 = use incremental PID on angular velocity
              0 = direct calculation (target_v_angle × FRAME_W_HALF × DEGREE_TO_RAD)

Line 21:    #define V_DEGREE_FROM_IMU   1      — IMU as angular velocity source
              1 = use imu_g_z (ICM-45686 gyro Z)
              0 = calculate from wheel encoder differential

Line 23:    #endif /* __RTOS_CONFIG_H__ */
```

---

## 8. Control Layer (continued)

### 8.3 Motor.h / Motor.c 鈥?Motor Driver

**Header:** `Core/Inc/Motor.h` (93 lines) | **Source:** `Core/Src/Motor.c` (416 lines)

#### Motor.h 鈥?Line-by-Line

```
Line 1-11:  Include guard + extern "C" + main/tim/gpio/pid includes

Line 16:    #define RTOS_MOTOR_TIME_ITV (TASK_ITV_CAR*0.001F)
              鈥?Control interval in seconds: 10ms * 0.001 = 0.01s (100 Hz)
Line 17:    #define RTOS_MOTOR_TIME_ITV_REC (1.0F/RTOS_MOTOR_TIME_ITV)
              鈥?Reciprocal: 1/0.01s = 100 Hz (speed conversion factor)
Line 20:    #define V_ENC_TO_REAL (1.0F/V_REAL_TO_ENC)
              鈥?Encoder鈫抮eal speed conversion: 1/7.136 鈮?0.140 mm/enc

Line 22-48: typedef struct __motor 鈥?Per-motor control struct:
Line 25:      GPIO_TypeDef* motor_gpio1;   鈥?H-bridge control pin 1 port
Line 26:      uint16_t      motor_pin1;    鈥?H-bridge control pin 1 number
Line 27:      GPIO_TypeDef* motor_gpio2;   鈥?H-bridge control pin 2 port
Line 28:      uint16_t      motor_pin2;    鈥?H-bridge control pin 2 number
               H-bridge truth table:
               pin1=SET, pin2=RESET 鈫?Forward
               pin1=RESET, pin2=SET 鈫?Backward
               pin1=RESET, pin2=RESET 鈫?Stop (coast)
Line 31:      TIM_HandleTypeDef *htim;     鈥?PWM timer handle (unused with SW PWM)
Line 32:      uint32_t Channel;            鈥?Software PWM channel (0-3 鈫?dutyA/B/C/D)
Line 33:      uint32_t dutyCycle;          鈥?Absolute duty cycle value
Line 36:      _Bool dir;                   鈥?Polarity: 1=forward, 0=reverse (physical wiring)
Line 39:      pid v_pid;                   鈥?Per-motor speed PID controller instance
Line 42:      int EncSource;               鈥?Encoder pulse delta (since last update)
Line 43:      int total_enc;               鈥?Cumulative encoder count (odometry)
Line 44:      float v_enc;                 鈥?Current encoder speed (enc/s)
Line 45:      float v_real;                鈥?Current real speed (mm/s)
Line 46:      float target_v_enc;          鈥?Target encoder speed (enc/s)
Line 47:      float duty;                  鈥?Signed PWM duty cycle (positive=forward)

Line 52-55: typedef enum _motor_choice { LEFT=0, RIGHT }
              鈥?Side selector for wheel API functions

Line 57-60: extern motor variable declarations:
            extern motor motor_LeftFront;     鈥?Left front motor instance
            extern motor motor_LeftRear;      鈥?Left rear motor instance
            extern motor motor_RightFront;    鈥?Right front motor instance
            extern motor motor_RightRear;     鈥?Right rear motor instance

Line 66-69: Motor control helpers:
Line 66:    void Motor_Forward(motor* motor)     鈥?Set GPIOs for forward rotation
Line 67:    void Motor_Backward(motor* motor)    鈥?Set GPIOs for reverse rotation
Line 68:    void Motor_Stop(motor* motor)        鈥?Stop motor (both pins low)
Line 69:    void Motor_SetPWM(motor* motor)      鈥?Apply duty cycle to PWM channel

Line 72-73: Utility functions:
Line 72:    float myabs(float num)               鈥?Absolute value
Line 73:    float duty_limit(float duty)         鈥?Clamp duty to 卤MAX_MOTOR_DUTY

Line 77-87: Public API functions:
Line 77:    void init_motor(void)                    鈥?Init all 4 motor structs
Line 78:    void Motor_Update_Input_All(void)        鈥?Update speeds from encoders
Line 79:    void Motor_Update_Output_All(void)       鈥?Run PID + set PWM outputs
Line 80:    void Motor_Set_V_Real_All(lf,lr,rf,rr)  鈥?Set 4 target speeds (mm/s)
Line 81:    float Wheel_Get_V_Real(choice)          鈥?Get left/right speed (uses best encoder)
Line 82:    void Wheel_Set_V_Real(left,right)       鈥?Set left/right target speed
Line 83:    float Wheel_Get_Distance(choice)        鈥?Get left/right odometry (mm)
Line 84:    void Wheel_Clear_Distance(void)         鈥?Reset all odometry accumulators
Line 85:    void Motor_Set_V_Real(motor*, float)    鈥?Set single motor target speed (mm/s)
Line 86:    void Motor_Set_V_Enc(motor*, float)     鈥?Set single motor target speed (enc/s)
```

#### Motor.c 鈥?Line-by-Line

```
Line 1-4:   Includes: motor.h, config.h, spi.h, PWM.h

--- Global Motor Instances (lines 7-16) ---
Line 8:     motor motor_LeftFront = {0}      鈥?Zero-initialize LF motor
Line 9:     motor motor_LeftRear = {0}       鈥?Zero-initialize LR motor
Line 10:    motor motor_RightFront = {0}     鈥?Zero-initialize RF motor
Line 11:    motor motor_RightRear = {0}      鈥?Zero-initialize RR motor
Line 12:    extern uint16_t dutyA,B,C,D      鈥?Access PWM threshold variables
Line 15-16: motor* wheel_left / wheel_right  鈥?Pointers to best encoder per side

--- Static forward declarations (lines 21-29) ---
            Motor_Update_Output, Motor_Update_Input, Motor_Get_V_Real,
            Motor_Clear_Distance, Motor_Get_Distance, Motor_Judge_Accuracy

--- init_motor (lines 49-102) ---
Line 49-102: void init_motor(void):
              Configures all 4 motor structs with hardcoded GPIO pin assignments:

Motor GPIO Pinout:
  LEFT FRONT:   dir1=PE3, dir2=PC13 鈫?Channel 0 (dutyA)
  LEFT REAR:    dir1=PC3, dir2=PC0  鈫?Channel 1 (dutyB)
  RIGHT FRONT:  dir1=PB1, dir2=PE7  鈫?Channel 3 (dutyD) [NOTE: D=RF]
  RIGHT REAR:   dir1=PC5, dir2=PC4  鈫?Channel 2 (dutyC) [NOTE: C=RR]

Line 87-90: All motors set dir=1 (forward polarity)
Line 93-101: Set PID limits + P/I/D coefficients from motor_config.h macros

--- Per-Motor Functions (lines 109-209) ---

Line 109-112: void Motor_Set_V_Real(motor* motor, float v_real):
              Convert v_real to v_enc via V_REAL_TO_ENC conversion factor
              Call Motor_Set_V_Enc(motor, v_enc)

Line 119-121: float Motor_Get_V_Real(motor* motor):
              return motor->v_real (current speed mm/s)

Line 128-132: void Motor_Set_V_Enc(motor* motor, float v_enc):
Line 129-130: Clamp v_enc to 卤MAX_V_ENC (prevents unreasonable targets)
Line 131:    motor->target_v_enc = v_enc       鈥?Store target

Line 140-175: void Motor_Update_Output(motor* motor):
Line 147:   temp = PID_Cal_Inc(&motor->v_pid, myabs(v_enc), myabs(target_v_enc))
              鈥?Incremental PID on ABSOLUTE speed values
              鈥?PID works on magnitude; direction controlled separately
Line 148:   motor->duty = temp                  鈥?Signed duty from PID
Line 151-152: motor->dutyCycle = myabs(duty); Motor_SetPWM(motor)
              鈥?Apply absolute duty to PWM channel

Direction control logic (lines 154-172):
  if motor->dir == 1 (normal forward polarity):
    target_enc > 0 鈫?Motor_Forward (SET, RESET)
    target_enc < 0 鈫?Motor_Backward (RESET, SET)
  if motor->dir == 0 (reversed polarity):
    target_enc > 0 鈫?Motor_Backward (RESET, SET) 鈥?reversed!
    target_enc < 0 鈫?Motor_Forward (SET, RESET) 鈥?reversed!

Line 181-191: void Motor_Update_Input(motor* motor):
Line 182-183: if (!motor->dir): EncSource = -EncSource 鈥?Flip sign for reversed polarity
Line 187:    motor->v_enc = EncSource / (TASK_ITV_CAR * 0.001)
              鈥?Convert pulse delta to enc/s: divide by 0.01s interval
Line 188:    motor->v_real = v_enc * V_ENC_TO_REAL
              鈥?Convert to mm/s for display/odometry
Line 189:    motor->total_enc += EncSource      鈥?Accumulate for distance

Line 197-199: void Motor_Clear_Distance(motor* motor):
              motor->total_enc = 0             鈥?Reset odometry

Line 205-209: float Motor_Get_Distance(motor* motor):
              return total_enc / V_REAL_TO_ENC 鈥?Convert enc pulses to mm

--- Accuracy Selection (lines 215-230) ---
Line 215-230: void Motor_Judge_Accuracy(void):
              #if USE_4_MOTOR (4WD mode):
                Left side: compare LF vs LR tracking error 鈫?pick better as wheel_left
                Right side: compare RF vs RR tracking error 鈫?pick better as wheel_right
              This dynamically selects the encoder with less wheel slip.

--- Batch Functions (lines 239-293) ---
Line 239-248: void Motor_Update_Input_All(void):
              #if USE_4_MOTOR: update LR + RR; always update LF + RF
              Motor_Judge_Accuracy() 鈫?select best encoder per side

Line 254-261: void Motor_Update_Output_All(void):
              #if USE_4_MOTOR: update LR + RR; always update LF + RF

Line 270-277: void Motor_Set_V_Real_All(lf, lr, rf, rr):
              #if USE_4_MOTOR: also set rear motors

Line 286-293: void Motor_Set_V_Enc_All(...):
              Encoder speed batch set (rarely used directly)

--- Wheel Interface Functions (lines 301-342) ---
Line 301-308: float Wheel_Get_V_Real(_motor_choice choice):
              if LEFT 鈫?wheel_left->v_real; else 鈫?wheel_right->v_real

Line 315-317: void Wheel_Set_V_Real(float left, float right):
              Calls Motor_Set_V_Real_All(left, left, right, right)
              鈥?Both left motors get same target; both right motors get same target

Line 324-331: float Wheel_Get_Distance(_motor_choice choice):
              if LEFT 鈫?Motor_Get_Distance(wheel_left)
              else    鈫?Motor_Get_Distance(wheel_right)

Line 337-342: void Wheel_Clear_Distance(void):
              Clear total_enc on all 4 motors

--- Utility Functions (lines 353-368) ---
Line 353-355: float myabs(float num):
              return num > 0 ? num : -num

Line 363-368: float duty_limit(float duty):
              Clamp to 卤MAX_MOTOR_DUTY (0.999)

--- H-Bridge GPIO Control (lines 384-413) ---
Line 384-390: void Motor_Forward(motor* motor):
              HAL_GPIO_WritePin(gpio1, pin1, SET)
              HAL_GPIO_WritePin(gpio2, pin2, RESET)

Line 392-397: void Motor_Backward(motor* motor):
              HAL_GPIO_WritePin(gpio1, pin1, RESET)
              HAL_GPIO_WritePin(gpio2, pin2, SET)

Line 399-403: void Motor_Stop(motor* motor):
              HAL_GPIO_WritePin(gpio1, pin1, RESET)
              HAL_GPIO_WritePin(gpio2, pin2, RESET)
              鈥?Both pins low = H-bridge coast/brake

Line 405-413: void Motor_SetPWM(motor* motor):
              Set_PWM_Duty((uint8_t)Channel, duty)
              鈥?Write duty to software PWM channel variable
```

### 8.4 car_attitude.h / car_attitude.c 鈥?Car Attitude

**Header:** `Core/Inc/car_attitude.h` (49 lines) | **Source:** `Core/Src/car_attitude.c` (148 lines)

#### car_attitude.h

```
Line 1-15:  Author cbr + include guard
Line 15:    #include "pid.h"
Line 19:    #define FRAME_W_HALF_REC (1.0F/FRAME_W_HALF)
              鈥?Reciprocal of half track width 鈮?0.0151 mm鈦宦?              Used for converting v_z (mm/s) to angular velocity (rad/s)

Line 22-33: typedef struct __car_attitude:
Line 24:    float current_v_line     鈥?Current linear velocity (mm/s)
Line 25:    float current_v_angle    鈥?Current angular velocity (deg/s)
Line 27:    float target_v_line      鈥?Target linear velocity (mm/s)
Line 28:    float target_v_angle     鈥?Target angular velocity (deg/s)
Line 30:    _Bool flag_stop          鈥?1 = car stopped
Line 31:    _Bool updated             鈥?Freshness flag (set to 1 on update)
Line 32:    pid pid_v_angle          鈥?Angular velocity PID controller

Line 36:    extern _car_attitude car_attitude

Line 38-46: Function prototypes:
Line 38:    Car_Attitude_Update_Output() 鈥?Angular vel PID 鈫?left/right speeds
Line 39:    Car_Attitude_Update_Input()  鈥?Speeds 鈫?v_line, v_angle (IMU/encoders)
Line 40:    Set_Car_Attitude(v_line, v_angle) 鈥?Set target speeds
Line 41:    init_Car_Attitude()         鈥?Init PID params
Line 43:    Set_Car_Stop()              鈥?flag_stop = 1
Line 44:    Set_Car_Start()             鈥?flag_stop = 0
Line 46:    Car_Attitude_Yaw_Update(v_angle, time) 鈥?Update yaw accumulator
```

#### car_attitude.c

```
Line 1-10:  Author copyright header (file name mis-labeled "car_control.c" in source)
Line 12-17: Includes: math.h, car_attitude.h, app.h, config.h, motor.h, IMU.h
Line 18:    _car_attitude car_attitude = {0}   鈥?Global attitude struct (zero init)
Line 19:    car_state_t car_state = {0}         鈥?Shared car state (zero init)

Line 26-32: void init_Car_Attitude(void):
Line 28:    Set_PID_Limit(&car_attitude.pid_v_angle,
                            LIMIT_INC_V_ANGLE, LIMIT_POS_V_ANGLE, LIMIT_ITGR_V_ANGLE)
Line 30:    Set_PID(&car_attitude.pid_v_angle, P_V_ANGLE, I_V_ANGLE, D_V_ANGLE)

Line 38-49: void Car_Attitude_Update_Input(void):
Line 39:    car_attitude.updated = 1             鈥?Mark data as fresh
Line 41:    left = Wheel_Get_V_Real(LEFT)        鈥?Get left wheel speed (mm/s)
Line 42:    right = Wheel_Get_V_Real(RIGHT)      鈥?Get right wheel speed (mm/s)
Line 43:    current_v_line = 0.5F 脳 (left + right) 鈥?Average = linear speed
Line 44-48: Angular velocity source selection:
              #if V_DEGREE_FROM_IMU (default: 1):
                current_v_angle = imu_g_z     鈥?Direct IMU Z gyro (deg/s)
              #else:
                current_v_angle = 0.5 脳 (right-left) 脳 FRAME_W_HALF_REC 脳 RAD_TO_DEGREE
                鈥?Encoder differential 鈫?angular velocity (deg/s)

Line 55-74: void Car_Attitude_Update_Output(void):
Line 59-62: if (flag_stop): v_left = v_right = v_z = 0 鈥?Stop: all zeros
Line 64-69: #if V_ANGLE_PID (default: 1):
              v_z = PID_Cal_Pos(pid_v_angle, current_v_angle, target_v_angle)
              鈥?Angular velocity PID: compensate disturbances
            #else:
              v_z = target_v_angle 脳 FRAME_W_HALF 脳 DEGREE_TO_RAD
              鈥?Direct calculation (no closed-loop compensation)
Line 70-71: v_left = target_v_line - v_z
            v_right = target_v_line + v_z
              鈥?Differential drive: add v_z to right, subtract from left
Line 73:    Wheel_Set_V_Real(v_left, v_right)    鈥?Apply to motors

Line 81-105: void Set_Car_Attitude(float v_line_target, float v_angle_target):
Line 83-84: Clamp v_line_target to 卤MAX_V_REAL (鈭?97.4 mm/s)
Line 87-88: Clamp v_angle_target to 卤MAX_V_ANGLE (60 deg/s)
Line 91-92: Store targets
Line 95-99: If both targets == 0 鈫?Set_Car_Stop(); else 鈫?Set_Car_Start()
Line 102-104: #if !USE_CAR_CONTROL 鈫?PID_Clear(&pid_v_angle)
              鈥?Reset angular PID when bypassing position loop

Line 112-122: Set_Car_Stop() / Set_Car_Start():
              flag_stop = 1 or 0

Line 135-148: void Car_Attitude_Yaw_Update(float v_angle, float time):
Line 137:   car_state.yaw += v_angle 脳 time       鈥?Integrate angular velocity
Line 138:   car_state.yaw_all += v_angle 脳 time    鈥?Accumulate all revolutions
Line 140-147: Yaw wrap-around handling:
              if yaw < 0:  yaw += 360; yaw_circles--  鈥?CCW overflow
              if yaw 鈮?360: yaw -= 360; yaw_circles++ 鈥?CW overflow
              Maintains yaw in [0, 360) range
```

### 8.5 car_control.h / car_control.c 鈥?Car Control

**Header:** `Core/Inc/car_control.h` (75 lines) | **Source:** `Core/Src/car_control.c` (231 lines)

#### car_control.h

```
Line 1-6:   Include guard + main.h + stdbool.h + pid.h

Line 8-11:  Tolerance constants:
Line 8:     #define BIAS_LINE   10.0F           鈥?Linear distance bias for overshoot (mm)
Line 9:     #define BIAS_ANGLE  3.0F            鈥?Angular bias for overshoot (deg)
Line 10:    #define SPIN_INT_RATE   0.5F         鈥?Interrupt tolerance = 50% of remaining spin
Line 11:    #define LINE_INT_RATE   0.5F         鈥?Interrupt tolerance = 50% of remaining line

Line 13-19: typedef enum __car_mode:
Line 14:    __car_mode_DISABLE = 0   鈥?Bypass position PID (direct attitude control)
Line 15:    STOP                     鈥?Stationary
Line 16:    GO_LINE                  鈥?Move straight line
Line 17:    TO_POINT                 鈥?Arc to target (x,y) point
Line 18:    SPIN                     鈥?Rotate in place by angle

Line 26-32: typedef struct __to_point_parameter:
Line 28:    float dir                 鈥?Arc direction: +1 = CCW, -1 = CW
Line 29:    float R                   鈥?Arc radius (mm)
Line 30:    float v_bias              鈥?Preset velocity bias
Line 31:    float interrupt_tolerance 鈥?Distance after which operation is interruptible

Line 39-46: typedef struct __spin_parameter:
Line 41:    float r                   鈥?Spin radius (can be 0 for in-place)
Line 42:    int16_t circles           鈥?Yaw circle counter
Line 43:    float start_yaw           鈥?Yaw at start of spin operation
Line 45:    float interrupt_tolerance 鈥?Angle after which operation is interruptible

Line 48-62: typedef struct __car_control:
Line 50:    _car_mode mode             鈥?Current control mode
Line 51-52: float target/current_line_distance 鈥?Distance tracking (mm)
Line 53:    pid pid_line_pos           鈥?Position PID for linear motion
Line 54-55: float target/current_spin_angle 鈥?Angle tracking (deg)
Line 56:    pid pid_spin               鈥?Position PID for spinning
Line 57:    _to_point_parameter        鈥?Arc-specific parameters
Line 58:    _spin_parameter            鈥?Spin-specific parameters
Line 59:    bool oprate_done           鈥?TRUE when operation completed
Line 60:    bool if_enable_interrupt   鈥?TRUE when operation can be interrupted

Line 64:    extern _car_control car_control
Line 66-72: Function prototypes
```

#### car_control.c

```
Line 1-5:   Includes: app.h, car_attitude.h, car_control.h, config.h, math.h
Line 7:     _car_control car_control             鈥?Global control instance
Line 9-12:  Static forward declarations

Line 14-20: Upload ACK stubs:
Line 14:    extern void Upload_Car_IntEnableAck  鈥?Interrupt enable acknowledgment
Line 15:    extern void Upload_Car_OperateDoneAck 鈥?Done acknowledgment
Line 18-20: __weak empty stubs (override in app layer)

Line 26-31: void init_Car_Contorl(void):
Line 27-28: Set PID limits for pid_line_pos and pid_spin
Line 29-30: Set PID coefficients from control_config.h

Line 33-43: static void clear_car_control(void):
Line 34:    oprate_done = 0; if_enable_interrupt = 0
Line 37:    spin_parameter.start_yaw = car_state.yaw 鈥?Record starting angle
Line 39:    car_state.yaw_circles = 0              鈥?Reset circle counter
Line 40:    Wheel_Clear_Distance()                 鈥?Zero odometry
Line 41-42: PID_Clear(&pid_line_pos), PID_Clear(&pid_v_angle)
              鈥?Reset all PID states for new command

Line 58-99: void Set_Car_Control(float x, float y, float angle):
              Mode selection logic:

  if y==0 && x!=0 && angle==0:  鈫?GO_LINE
    target = x/2 (half per accumulator)
    interrupt_tolerance = target 脳 0.5
    add_bias(target, 0) 鈫?adds BIAS_LINE compensation

  if y!=0 && x!=0 && angle==0: 鈫?TO_POINT
    dir = y>0 ? 1:-1 (CCW/CW)
    R = (x虏+y虏)/(2y) (circle radius)
    arc_length = R 脳 asin(x/R) (chord鈫抋rc conversion)
    Same tolerance/bias as GO_LINE

  if x==0 && angle!=0:         鈫?SPIN
    target_spin_angle = angle
    r = |y/2| (can be 0 for in-place spin)
    interrupt_tolerance = |angle| 脳 0.5
    add_bias(target, 1) 鈫?adds BIAS_ANGLE compensation

  else: 鈫?STOP

Line 101-111: void Set_Car_Control_Absolute(float x, float y, float angle):
              SPIN to ABSOLUTE heading angle (not relative):
              target_spin_angle = (angle + 360 - (yaw_all % 360)) % 360
              鈥?Computes shortest-path relative angle from current heading

Line 116-160: void Car_Control_Update_Input(void):
              State machine checking progress for each mode:

  GO_LINE:
    get_current_distance() 鈫?avg of left+right wheel distance
    if error 鈮?BIAS_LINE 鈫?oprate_done=1, STOP
    if error < interrupt_tolerance 鈫?if_enable_interrupt=1

  TO_POINT:
    Same as GO_LINE (arc distance checked same way)

  SPIN:
    get_current_spin_angle() 鈫?yaw_circles脳360 + yaw - start_yaw
    if error 鈮?BIAS_ANGLE 鈫?oprate_done=1, STOP
    if error < interrupt_tolerance 鈫?if_enable_interrupt=1

Line 167-199: void Car_Control_Update_Output(void):
              #if USE_CAR_CONTROL (default 1):

  GO_LINE:
    target_v_line = PID_Cal_Pos(pid_line_pos, current_distance, target)
                    + v_bias
    Set_Car_Attitude(target_v_line, 0) 鈥?Pure linear motion

  TO_POINT:
    target_v_line = PID_Cal_Pos(pid_line_pos...) + v_bias
    target_v_angle = dir 脳 target_v_line / R 脳 RAD_TO_DEGREE
    鈥?Angular velocity needed for circular arc
    Set_Car_Attitude(target_v_line, target_v_angle)

  SPIN:
    target_v_angle = PID_Cal_Pos(pid_spin, current_angle, target_angle)
    target_v_line = |target_v_angle| 脳 DEGREE_TO_RAD 脳 r
    鈥?Linear velocity scaled to maintain pivot radius r
    Set_Car_Attitude(target_v_line, target_v_angle)

  default (STOP):
    Set_Car_Attitude(0, 0)

Line 201-203: static void get_current_distance(void):
              current_line_distance = 0.5 脳 (Wheel_Get_Distance(LEFT) +
                                            Wheel_Get_Distance(RIGHT))

Line 205-207: static void get_current_spin_angle(void):
              current_spin_angle = yaw_circles 脳 360 + yaw - start_yaw
              鈥?Total angular displacement including multi-circle overflow

Line 209-228: static float add_bias(float target, bool if_spin):
              Adds overshoot compensation in direction of travel:
              if_spin 鈫?target 卤 BIAS_ANGLE
              else    鈫?target 卤 BIAS_LINE
              鈥?Guarantees the target is always slightly overshot
```

**Control Loop Architecture Summary:**

```
Set_Car_Control(x,y,angle)
  鈫?car_control.mode = GO_LINE / TO_POINT / SPIN
car_control.target_line_distance / target_spin_angle
clear_car_control() 鈥?Reset PIDs + odometry
  鈫?[CarControl_Handler @ 10ms]
  鈫?Step 1: Tim_Update_Enc_Speed()               鈥?Read encoder pulse deltas
Step 2: Motor_Update_Input_All()             鈥?Encoders 鈫?v_enc, v_real
Step 3: Car_Attitude_Update_Input()          鈥?v_line, v_angle (IMU or encoders)
Step 4: Car_Control_Update_Input()           鈥?Check progress 鈫?oprate_done?
Step 5: Car_Control_Update_Output()          鈥?Position PID 鈫?target_v_line/v_angle
         鈹斺攢 PID_Cal_Pos(pid_line_pos/pid_spin)
         鈹斺攢 Set_Car_Attitude(v_line, v_angle)
Step 6: Car_Attitude_Update_Output()         鈥?Angular vel PID 鈫?v_left, v_right
         鈹斺攢 PID_Cal_Pos(pid_v_angle) or direct calc
         鈹斺攢 Wheel_Set_V_Real(v_left, v_right)
Step 7: Motor_Update_Output_All()            鈥?Speed PID 鈫?PWM duty
         鈹斺攢 PID_Cal_Inc(v_pid) 鈫?per motor
         鈹斺攢 Motor_Forward/Backward
         鈹斺攢 Motor_SetPWM 鈫?Set_PWM_Duty
  鈫?TIM6 ISR: Software PWM output to GPIO pins
```

---

## 9. Sensor Layer

### 9.1 IMU.h / IMU.c 鈥?IMU Driver & AHRS

**Header:** `Core/Inc/IMU.h` (24 lines) | **Source:** `Core/Src/IMU.c` (283 lines)

#### IMU.h

```
Line 1-5:   Include guard + math.h + M_PI definition (3.1415926535)
Line 6-11:  typedef struct { float x, y, z; } xyz_f_t 鈥?3D vector type
Line 12:    extern xyz_f_t north, west    鈥?Reference direction vectors
Line 13:    extern volatile float yaw[5]  鈥?Yaw angle ring buffer (5 samples)
Line 14:    extern float motion6[7]       鈥?Raw IMU data (defined main.c)
Line 15:    extern float imu_g_z          鈥?Z-axis gyro angular rate (deg/s)

Line 17-22: Function prototypes:
Line 17:    void IMU_init(void)                      鈥?Init ICM-45686 + set q0=1
Line 18:    void IMU_getYawPitchRoll(float *ypr)    鈥?Convert quaternion 鈫?Euler angles
Line 19:    void IMU_TT_getgyro(float *zsjganda)    鈥?Copy raw sensor data
Line 21:    void MPU6050_InitAng_Offset(void)        鈥?Legacy empty stub
```

#### IMU.c 鈥?Full Annotation

```
Line 1-11:  Author: lisn3188, "Implementation of mini IMU"
Line 12-18: Includes: IMU.h, usart.h, inv_imu_driver.h, config.h,
                      car_attitude.h, read_aux_data_mode.h, sys_time.h

--- Global Variables (lines 19-28) ---
Line 19:    xyz_f_t north, west                    鈥?Reference direction vectors
Line 20:    volatile float exInt, eyInt, ezInt    鈥?Gyro bias error integrals
Line 21:    volatile float q0, q1, q2, q3         鈥?Attitude quaternion:
              q0 = scalar (w), q1 = i, q2 = j, q3 = k
              Initial: q0=1, q1=q2=q3=0 (identity rotation)
Line 22:    volatile float integralFBhand, handdiff 鈥?Unused
Line 23:    volatile uint32_t lastUpdate, now      鈥?AHRS time tracking (us units)
Line 24:    volatile float yaw[5] = {0}            鈥?Filtered yaw ring buffer
Line 25:    int16_t Ax_offset, Ay_offset           鈥?Accelerometer offsets
Line 26:    float TTangles_gyro[7]                 鈥?Raw IMU data [a_x,a_y,a_z,g_x,g_y,g_z,temp]
Line 27:    float Angle_Final[3]                   鈥?Final Euler angles
Line 28:    float imu_g_z                          鈥?Z-axis gyro angular rate (deg/s)

--- Fast Inverse Square Root (lines 34-42) ---
Line 34-42: float invSqrt1(float x):
              Quake III fast inverse square root algorithm
              Magic constant: 0x5f3759df
              Used for quaternion and vector normalization
              Newton-Raphson refinement: y = y 脳 (1.5 - halfx 脳 y虏)

--- IMU_init (lines 46-61) ---
Line 46-61: void IMU_init(void):
Line 47:    if (0x00 == setup_imu(1, 1, 1))        鈥?Init ICM-45686:
              1 = use low-noise mode
              1 = enable accelerometer
              1 = enable gyroscope
Line 49-52: Initialize quaternion to identity: q0=1, q1=q2=q3=0
Line 53-55: exInt = eyInt = ezInt = 0            鈥?Clear gyro bias integrals
Line 56:    lastUpdate = now = nowtime            鈥?Init AHRS timestamps
Line 60:    printf("IMU ERROR!!\r\n")             鈥?Error on init failure

--- Gyro Variance Calculation (lines 63-103) ---
Line 63-67: Static gyro variance tracking:
Line 63:    static double Gyro_fill[3][300]        鈥?Ring buffer (300 samples 脳 3 axes)
Line 64:    static double Gyro_total[3]           鈥?Running sum per axis
Line 65:    static double sqrGyro_total[3]        鈥?Running sum of squares
Line 66:    static int GyroinitFlag = 0           鈥?0=filling, 1=running
Line 67:    static int GyroCount = 0              鈥?Current index

Line 70-103: void calGyroVariance(data[], length, sqrResult[], avgResult[]):
              Sliding window variance computation:
              While filling: accumulate totals for each axis
              While running: subtract old sample, add new sample (O(1) update)
              After GyroinitFlag=1, compute variance:
              avg = total/length
              sqrResult = (sqr_total - total虏/length) / length
              Detects if IMU is stationary: all 3 axes variance < 0.02

Line 104-105: float gyro_offset[3] = {0}          鈥?Auto-calibrated zero-rate offsets
              int CalCount = 0                    鈥?Stationarity counter

--- IMU_getValues (lines 112-152) ---
Line 112-152: void IMU_getValues(float *values):
Line 119:   bsp_IcmGetRawData(accgyroval, &accgyroval[3], &accgyroval[6])
              鈥?Read ICM-45686: accel mg 鈫?accgyroval[0..2],
                                 gyro dps 鈫?accgyroval[3..5],
                                 temp 掳C 鈫?accgyroval[6]
Line 120-126: Copy to TTangles_gyro[0..6]
Line 128:   calGyroVariance(&TTangles_gyro[3], 100, sqr, avg)
              鈥?100-sample sliding window variance on gyro data
Line 129-140: Auto gyro offset calibration:
              if all 3 axes variance < 0.02 AND CalCount 鈮?99:
                gyro_offset[0..2] = avgResult_gyro[0..2]
                Reset error integrals
                CalCount = 0
                if_get_offset = 1
              else if CalCount < 100: CalCount++
Line 141-146: Apply offsets: values[3..5] = raw_gyro - gyro_offset
              values[0..2] = raw_accel (no offset applied to accel here)
Line 147:   imu_g_z = values[5]                   鈥?Expose Z gyro for car_attitude
Line 148-151: if (if_get_offset):                   鈥?On initial calibration success:
              Car_Attitude_Yaw_Update(imu_g_z, TASK_ITV_IMU 脳 0.001)
              鈥?Begin yaw tracking once offsets are calibrated

--- IMU_AHRSupdate 鈥?Custom Mahony Filter (lines 161-234) ---
Line 161-162: #define Kp 0.5f, Ki 0.001f          鈥?PI controller gains

Line 164-234: void IMU_AHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz):
              gx/gy/gz = gyroscope (掳/s), ax/ay/az = accelerometer (g),
              mx/my/mz = magnetometer (normalized)

Algorithm steps:
  1. Pre-compute quaternion products (q0q0, q0q1, ..., q3q3)
  2. Compute halfT from time delta (now - lastUpdate) / 20000us
     鈥?20000us comes from 2脳10ms half-period assumption
     鈥?Handles 16-bit time overflow
  3. Normalize accelerometer and magnetometer with invSqrt1
  4. Estimate gravity direction from current quaternion:
     vx = 2(q1q3 - q0q2)
     vy = 2(q0q1 + q2q3)
     vz = q0虏 - q1虏 - q2虏 + q3虏
  5. Compute orientation error: cross(estimated_gravity, measured_accel)
     ex = (ay脳vz - az脳vy), ey = (az脳vx - ax脳vz), ez = (ax脳vy - ay脳vx)
  6. PI controller on gyro drift:
     if error 鈮?0: integrate (exInt += ex 脳 Ki 脳 halfT) for each axis
     Correct gyro: gx += Kp脳ex + exInt, etc.
  7. Quaternion integration (1st order):
     q0 += (-q1脳gx - q2脳gy - q3脳gz) 脳 halfT
     q1 += ( q0脳gx + q2脳gz - q3脳gy) 脳 halfT
     q2 += ( q0脳gy - q1脳gz + q3脳gx) 脳 halfT
     q3 += ( q0脳gz + q1脳gy - q2脳gx) 脳 halfT
  8. Normalize quaternion with invSqrt1

--- IMU_getQ (lines 242-253) ---
Line 242-253: void IMU_getQ(float *q):
Line 244:   IMU_getValues(mygetqval)               鈥?Get raw sensor data
Line 246-247: IMU_AHRSupdate(
              mygetqval[3]脳蟺/180, mygetqval[4]脳蟺/180, mygetqval[5]脳蟺/180,
              mygetqval[0..2], mygetqval[6..8])
              鈥?Convert gyro from 掳/s to rad/s before AHRS
Line 249-252: Copy q0..q3 to output array

--- IMU_getYawPitchRoll (lines 261-268) ---
Line 261-268: void IMU_getYawPitchRoll(float *angles):
Line 262-263: IMU_getQ(q) 鈫?get current quaternion
Line 265:   yaw   = -atan2(2q鈧乹鈧?2q鈧€q鈧? -2q鈧偮?2q鈧兟?1) 脳 180/蟺
Line 266:   pitch = -asin(-2q鈧乹鈧?2q鈧€q鈧? 脳 180/蟺
Line 267:   roll  = atan2(2q鈧俼鈧?2q鈧€q鈧? -2q鈧伮?2q鈧偮?1) 脳 180/蟺
              Standard quaternion 鈫?Euler (ZYX Tait-Bryan) conversion

--- IMU_TT_getgyro (lines 270-278) ---
Line 270-278: void IMU_TT_getgyro(float *data):
              Copy TTangles_gyro[0..6] to output array
              鈥?Raw sensor data: accel[3] + gyro[3] + temp[1]

--- MPU6050_InitAng_Offset (lines 280-282) ---
              Empty legacy stub (from MPU6050 era)
```

### 9.2 read_aux_data_mode.h / read_aux_data_mode.c 鈥?IMU Transport

**Header:** `Core/Inc/read_aux_data_mode.h` (38 lines) | **Source:** `Core/Src/read_aux_data_mode.c` (266 lines)

#### read_aux_data_mode.h

```
Line 1-7:   Include guard + extern "C"
Line 13-20: int setup_imu(int use_ln, int accel_en, int gyro_en)
              鈥?Initialize ICM-45686: configure SPI4 transport, full scale, ODR, filters
              鈥?Returns 0 on success, negative on error
Line 22-29: int bsp_IcmGetRawData(float accel_mg[3], float gyro_dps[3], float *temp_degc)
              鈥?Read raw sensor data, convert to engineering units
Line 32:     u8 SPI4_ReadWriteByte(u8 TxData)
              鈥?SPI4 byte-level exchange (defined in CubeMX spi.c)
```

#### read_aux_data_mode.c 鈥?Key Sections

```
Line 1-15:  Includes + IMU interface selection
Line 7:     #define ICM_USE_HARD_SPI               鈥?Hardware SPI (SPI4) selected
Line 8:     //#define ICM_USE_I2C                   鈥?I2C alternative disabled

Line 17-18:  UI_I2C = 0, UI_SPI4 = 1               鈥?Interface type constants
Line 20-21:  SPI_IMU_CS / SPI_IMU_nCS macros       鈥?CS pin control via HAL GPIO
Line 24:     #define INV_MSG(level,msg,...) printf("%d," msg "\r\n", __LINE__, ...)
              鈥?Debug logging macro (prints source line number)

Line 26:     static inv_imu_device_t  imu_dev      鈥?ICM-45686 driver device struct
Line 28-29:  discard_accel_samples / discard_gyro_samples
              鈥?Startup sample discard counters (reject transient data)

Line 34-41:  #define SI_CHECK_RC(rc)                  鈥?Error-checking macro:
              if (si_print_error_if_any(rc)) 鈫?log error, delay 100ms, return rc

Line 46-72:  int si_print_error_if_any(int rc):
              Translates InvenSense error codes to human-readable strings:
              INV_IMU_ERROR 鈫?"Unspecified error"
              INV_IMU_ERROR_TRANSPORT 鈫?transport error
              INV_IMU_ERROR_TIMEOUT 鈫?timeout
              INV_IMU_ERROR_BAD_ARG 鈫?invalid argument
              INV_IMU_ERROR_EDMP_BUF_EMPTY 鈫?EDMP buffer empty

Line 84-103: static int icm45686_read_regs(reg, buf, len):
              #if I2C: IICreadBytes(0x69, reg, len, buf)
              #if SPI:
                reg |= 0x80                        鈥?Set SPI read bit (MSB=1)
                CS low 鈫?send reg 鈫?read len bytes 鈫?CS high

Line 105-119: static uint8_t io_write_reg(reg, value):
              SPI: CS low 鈫?send reg 鈫?send value 鈫?CS high

Line 121-132: static int icm45686_write_regs(reg, buf, len):
              Loop over len: io_write_reg(reg + i, buf[i])

Line 135-247: int setup_imu(int use_ln, int accel_en, int gyro_en):
              Complete ICM-45686 initialization sequence:

  1. Transport setup (lines 142-161):
     imu_dev.transport.read_reg = icm45686_read_regs
     imu_dev.transport.write_reg = icm45686_write_regs
     imu_dev.transport.serif_type = UI_SPI4
     imu_dev.transport.sleep_us = delay_us

  2. Power-up delay: delay_ms(100)

  3. SPI slew rate config: 10ns (prevents DK-SMARTMOTION bus corruption)

  4. WHOAMI check (lines 176-183):
     inv_imu_get_who_am_i(&imu_dev, &whoami)
     Verify whoami == INV_IMU_WHOAMI 鈫?return -1 on mismatch

  5. inv_imu_soft_reset(&imu_dev)              鈥?Reset IMU registers to defaults

  6. Interrupt configuration (lines 194-204):
     Pin: HIGH polarity, PULSE mode, PUSH-PULL drive
     Interrupts: DRDY (data ready) enabled, all others disabled

  7. Sensor configuration:
     Accelerometer: 卤4G FS, 200 Hz ODR, BW = ODR/4 = 50 Hz
     Gyroscope: 卤1000 dps FS, 200 Hz ODR, BW = 50 Hz

  8. Clock selection: RCOSC for LP mode

  9. Power modes (lines 226-236):
     if use_ln: accel/gyro 鈫?Low-Noise mode
     else:      accel/gyro 鈫?Low-Power mode
     Respect accel_en/gyro_en flags

  10. Startup discard (lines 239-242):
      accel: (ACC_STARTUP_TIME / 20000) + 1 samples
      gyro:  (GYR_STARTUP_TIME / 20000) + 1 samples

Line 248-264: int bsp_IcmGetRawData(accel_mg[3], gyro_dps[3], *temp_degc):
Line 253:   inv_imu_get_register_data(&imu_dev, &d)
              鈥?Read all 7 sensor registers at once (FIFO or raw registers)
Line 254:   SI_CHECK_RC(rc)
Line 256-258: Accel conversion: raw 脳 4 mg / 32.768
              鈥?ICM-45686: 16-bit scaled, 卤4G full scale
Line 259-261: Gyro conversion: raw 脳 1000 dps / 32768.0
              鈥?ICM-45686: 16-bit signed, 卤1000 dps full scale
Line 262:   Temp conversion: 25 + raw / 128.0 掳C
              鈥?ICM-45686 temperature formula
```

---

## 10. Common Layer

### 10.1 delay.h / delay.c 鈥?DWT Delay

**Header:** `Core/Inc/delay.h` (20 lines) | **Source:** `Core/Src/delay.c` (78 lines)

#### delay.h

```
Line 1-11:  Doxygen: DWT-based delay for FreeRTOS. SysTick NOT touched.
Line 14:    #include "main.h"                     鈥?HAL dependencies
Line 16:    void delay_init(uint32_t cpu_freq_mhz) 鈥?Init DWT cycle counter with CPU freq
Line 17:    void delay_ms(uint32_t nms)            鈥?Millisecond delay (spin-wait)
Line 18:    void delay_us(uint32_t nus)            鈥?Microsecond delay (spin-wait)
```

#### delay.c

```
Line 1-9:   File header: 480 MHz CPU 鈫?~2.08 ns / ~8.9 s overflow
Line 10:    #include "delay.h"
Line 12:    static uint32_t g_cpu_freq_mhz = 480U  鈥?Default CPU frequency (MHz)
Line 13:    static uint32_t g_cpu_freq_hz  = 480000000U 鈥?Default CPU frequency (Hz)

Line 22-37: void delay_init(uint32_t cpu_freq_mhz):
Line 24:    g_cpu_freq_mhz = cpu_freq_mhz;         鈥?Store frequency in MHz
Line 25:    g_cpu_freq_hz = cpu_freq_mhz 脳 1000000U; 鈥?Store in Hz
Line 28:    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
              鈥?Ensure SysTick uses HCLK (required before osKernelStart)
Line 31-34: if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U)
              CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
              鈥?Enable DWT trace unit (if not already enabled by debugger)
Line 35:    DWT->CYCCNT = 0U;                     鈥?Reset cycle counter to zero
Line 36:    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;   鈥?Enable DWT cycle counter

Line 46-58: void delay_us(uint32_t nus):
Line 48:    uint32_t start = DWT->CYCCNT;          鈥?Snapshot start count
Line 49:    uint32_t target = nus 脳 g_cpu_freq_mhz; 鈥?ticks = us 脳 MHz
              Example: 100us 脳 480MHz = 48000 cycles
Line 52:    if (nus == 0U) return;                 鈥?Zero delay = no-op
Line 54-57: do { cnt = DWT->CYCCNT; } while ((cnt - start) < target);
              鈥?Spin-wait loop using unsigned arithmetic (overflow-safe)
              鈥?(cnt - start) correctly handles 32-bit counter wrap-around

Line 67-78: void delay_ms(uint32_t nms):
Line 69:    if (nms == 0U) return;
Line 72-77: while (nms 鈮?1U):
              uint32_t chunk = (nms > 100U) ? 100U : nms;
              delay_us(chunk 脳 1000U);
              nms -= chunk;
              鈥?Chunked delay to allow interrupts between 100ms sub-delays
              鈥?Each 100ms chunk = 100000 脳 480MHz = 48 million cycles
```

### 10.2 sys_time.h / sys_time.c 鈥?System Time

**Header:** `Core/Inc/sys_time.h` (35 lines) | **Source:** `Core/Src/sys_time.c` (21 lines)

#### sys_time.h

```
Line 1-8:   Doxygen: system time interface, decouples global nowtime
Line 13:    #include <stdint.h>
Line 19:    extern uint32_t nowtime;              鈥?System time in 100us units
              Originally defined in main.c; migrated here for decoupling
Line 24:    uint32_t SysTime_GetMs(void)          鈥?Get milliseconds since boot
Line 29:    uint32_t SysTime_GetUs(void)          鈥?Get microseconds since boot
```

#### sys_time.c

```
Line 1-9:   File header + includes
Line 9:     uint32_t nowtime = 0                鈥?Global system time (100us units)
              Updated externally (presumably in TIM6 ISR or timer callback)
              Used by IMU AHRS for time delta calculation (lastUpdate/now)
Line 11:    static uint32_t cpu_freq_hz = 480000000

Line 13-16: uint32_t SysTime_GetMs(void):
              return HAL_GetTick()                鈥?Standard HAL tick (1ms resolution)

Line 18-21: uint32_t SysTime_GetUs(void):
              return DWT->CYCCNT / (cpu_freq_hz / 1000000)
              鈥?Divide 480MHz cycle count by 480 to get microseconds
```

---

## 11. Device Layer

### 11.1 uart_fifo.h / uart_fifo.c 鈥?UART Frame Protocol

**Header:** `Core/Inc/uart_fifo.h` (44 lines) | **Source:** `Core/Src/uart_fifo.c` (142 lines)

#### uart_fifo.h

```
Line 1-8:   Includes: stm32h7xx_hal.h, usart.h, FreeRTOS.h, queue.h, string.h

Line 10-22: #ifndef __UART_FRAME_RX_H:
Line 13-14: #include <stdint.h> / <stddef.h>
Line 17:    #define UART_FRAME_MAX_LEN 64            鈥?Max line length for UART3/4 frames
Line 20:    void uart_frame_rx_init(void)            鈥?Init interrupt reception for UART3/4

Line 25-27: Legacy frame protocol defines:
Line 25:    #define FRAME_HEADER 0xAA                鈥?Frame start marker
Line 26:    #define FRAME_TAIL   0x55                鈥?Frame end marker
Line 27:    #define MAX_FRAME_SIZE 256               鈥?Max frame payload in bytes

Line 30-35: Global variables (legacy USART1 frame protocol):
Line 30:    extern UART_HandleTypeDef huart1
Line 32:    extern uint8_t uart_rx_byte             鈥?UART ISR RX byte buffer
Line 33:    extern uint8_t uart_received_frame[MAX_FRAME_SIZE 脳 8]
              鈥?Frame data converted to individual bits
Line 34:    extern uint16_t uart_received_frame_length 鈥?Bit length of received frame
Line 35:    extern uint8_t uart_tx_buffer[MAX_FRAME_SIZE+2] 鈥?TX buffer with header/tail
Line 36:    extern uint16_t uart_tx_length           鈥?TX buffer length

Line 39-41: Function declarations:
Line 39:    void UART_Init(void)                     鈥?Start USART1 receive
Line 40:    void UART_SendFrame(uint8_t *bits, uint16_t len) 鈥?Encode + send frame
Line 41-42: void HAL_UART_RxCpltCallback / HAL_UART_TxCpltCallback
```

#### uart_fifo.c 鈥?Line-by-Line

```
Line 1-2:   Includes: uart_fifo.h, stdio.h

--- UART3/4 Frame Receive Variables (lines 4-12) ---
Line 4:     extern QueueHandle_t uart3_frame_queue   鈥?FreeRTOS queue (from freertos.c)
Line 5:     extern QueueHandle_t uart4_frame_queue
Line 7-12:  Static receive buffers:
Line 7:     static uint8_t rx4_frame_buf[64]         鈥?UART4 line buffer
Line 8:     static size_t frame4_idx = 0             鈥?UART4 buffer index
Line 9:     static uint8_t rx4_byte                  鈥?UART4 single-byte RX
Line 10-12: Same for UART3 (rx3_frame_buf, frame3_idx, rx3_byte)

--- uart_frame_rx_init (lines 14-30) ---
Line 14-30: void uart_frame_rx_init(void):
Line 16-19: __HAL_UART_CLEAR_OREFLAG/FEFLAG/NEFLAG/PEFLAG(&huart3)
              鈥?Clear UART3 error flags: Overrun, Framing, Noise, Parity
Line 20-23: Same error flag clear for UART4
Line 24-25: frame4_idx = 0; memset(rx4_frame_buf, 0, 64)
              鈥?Reset UART4 buffer state
Line 26:    HAL_UART_Receive_IT(&huart4, &rx4_byte, 1)
              鈥?Start UART4 interrupt reception (1 byte at a time, "bt" module)
Line 27-29: Same reset + init for UART3 ("cat" module)

--- Legacy USART1 Variables (lines 33-44) ---
Line 33:    extern UART_HandleTypeDef huart1
Line 34:    uint8_t uart_rx_byte                    鈥?USART1 RX byte
Line 37-40: uint8_t uart_received_frame[MAX_FRAME_SIZE 脳 8]
            uint16_t uart_received_frame_length = 0
            uint8_t uart_tx_buffer[MAX_FRAME_SIZE + 2]
            uint16_t uart_tx_length = 0

Line 42-44: void UART_Init(void):
              HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1) 鈥?Start USART1 reception

--- HAL_UART_RxCpltCallback (lines 49-112) ---
Line 49-112: void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart):

  USART1 handler (lines 51-75): Legacy bit-based frame protocol
Line 51-52: if (huart->Instance == USART1):
              static uint8_t frame_data[MAX_FRAME_SIZE] 鈥?Payload buffer
              static uint16_t frame_index = 0           鈥?Payload index
Line 55-56: if (rx_byte == FRAME_HEADER 0xAA):
              frame_index = 0                           鈥?Reset on frame start
Line 59-61: if (frame_index < sizeof(frame_data)):
              frame_data[frame_index++] = rx_byte       鈥?Accumulate payload
Line 63-71: if (rx_byte == FRAME_TAIL 0x55):
              Convert bytes to bits: for each byte, extract each bit:
              uart_received_frame[i脳8+bit] = (byte >> (7-bit)) & 0x01
Line 74:    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1) 鈥?Continue reception

  USART3 handler (lines 76-94): Line-based string protocol
Line 76-94: if (huart->Instance == USART3):
Line 79:    if (rx3_byte == '\n' || frame3_idx 鈮?63):
              frame terminated by newline or buffer full
Line 80:      rx3_frame_buf[frame3_idx] = '\0'       鈥?Null-terminate string
Line 84-88:    xQueueSendFromISR(uart3_frame_queue, rx3_frame_buf, &xHigherPriorityTaskWoken)
              鈥?Send complete line to UART3 processing task (ISR-safe)
              frame3_idx = 0; memset(...)            鈥?Reset buffer
              portYIELD_FROM_ISR(xHigherPriorityTaskWoken) 鈥?Context switch if needed
Line 89:    else if (rx3_byte != '\r'):              鈥?Skip carriage return
              rx3_frame_buf[frame3_idx++] = rx3_byte  鈥?Append to buffer
Line 93:    HAL_UART_Receive_IT(&huart3, &rx3_byte, 1) 鈥?Continue

  UART4 handler (lines 95-111): Identical structure to USART3 handler
              Delivers lines to uart4_frame_queue

--- UART_SendFrame (lines 114-136) ---
Line 114-136: void UART_SendFrame(uint8_t *bit_data, uint16_t bit_length):
              Legacy bit-based frame transmitter:
Line 115-118: if bit_length % 8 鈮?0 鈫?error, return
Line 120-128: Convert bits back to bytes:
              For each 8-bit group, reconstruct byte by shifting bits
Line 122:   uart_tx_buffer[0] = FRAME_HEADER 0xAA   鈥?Insert header
Line 131:   uart_tx_buffer[byte_length+1] = FRAME_TAIL 0x55 鈥?Insert tail
Line 133-135: HAL_UART_Transmit_IT(&huart1, tx_buffer, length) 鈥?Send via UART1

--- HAL_UART_TxCpltCallback (lines 138-142) ---
Line 138-142: void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart):
              if (huart->Instance == USART1): printf("UART Transmission Complete!\n")
```

### 11.2 keyboard.h / keyboard.c 鈥?Matrix Keyboard

**Header:** `Core/Inc/keyboard.h` (51 lines) | **Source:** `Core/Src/keyboard.c` (111 lines)

#### keyboard.h

```
Line 1-12:  Include guard + extern "C" + HAL + FreeRTOS includes

Line 16-32: Matrix pin definitions:
            ROW1: PE9,  ROW2: PE11,  ROW3: PE13,  ROW4: PE14
            COL1: PA10, COL2: PB5,   COL3: PC9,   COL4: PC8

Line 35-37: Configuration constants:
Line 35:    #define KEY_SCAN_INTERVAL_MS   5       鈥?5 ms between scan cycles
Line 36:    #define DEBOUNCE_TICKS         pdMS_TO_TICKS(20) 鈥?20 ms debounce
Line 37:    #define KEY_QUEUE_LENGTH       10      鈥?Key event queue capacity

Line 40:    extern osMessageQueueId_t queue_keyHandle  鈥?Key press queue
Line 42-44: Function prototypes:
Line 42:    void Key_Init(void)                    鈥?Key init (not implemented)
Line 43:    void Key_StartScanTask(void)           鈥?Create FreeRTOS key scan task
Line 44:    void key_callback1(void)               鈥?Callback for external trigger
```

#### keyboard.c

```
Line 1-2:   Includes: keyboard.h, main.h

Line 5-6:   #define ROW_NUM 4, COL_NUM 4          鈥?4脳4 matrix

Line 9-14:  static const char KEY_MAP[4][4] = {    鈥?Key character map:
              {'1', '2', '3', 'A'},
              {'4', '5', '6', 'B'},
              {'7', '8', '9', 'C'},
              {'*', '0', '#', 'D'}

Line 17-35: Pin configuration arrays:
Line 17-25: ROWS[] = {{PE9},{PE11},{PE13},{PE14}} 鈥?Row pin array
Line 27-35: COLS[] = {{PA10},{PB5},{PC9},{PC8}}  鈥?Column pin array

Line 42-46: void Key_StartScanTask(void):
            xTaskCreate(vKeyScanTask, "KeyScan",
                        configMINIMAL_STACK_SIZE 脳 2,  鈥?128脳2 = 256 words = 1024 B
                        NULL, tskIDLE_PRIORITY + 2, NULL)
            鈥?Creates a separate FreeRTOS task for key scanning

Line 50-100: static void vKeyScanTask(void *pvParams):
Line 51-53: Static state tracking variables:
Line 51:    static TickType_t xLastTick[4][4] = {0} 鈥?Last debounce time per key
Line 52:    static uint8_t keyState[4][4] = {0}    鈥?0=idle, 1=debouncing, 2=pressed

Line 54:    for(;;) {
Line 56-92: For each row (0-3):
Line 58:      HAL_GPIO_WritePin(ROWS[row], ROWS[row].pin, RESET)
              鈥?Drive current row LOW (active)
Line 59:      vTaskDelay(pdMS_TO_TICKS(1))          鈥?1ms settling time

Line 61-91: For each column (0-3):
Line 63:      GPIO_PinState state = HAL_GPIO_ReadPin(COLS[col])
              鈥?Read column state (LOW = key pressed)

              State machine:
              CASE 0 (idle):
                if state == LOW:                   鈥?Key just pressed
                  keyState = 1; record tick        鈥?Begin debounce
              CASE 1 (debouncing):
                if (current_tick - last_tick) > DEBOUNCE_TICKS (20ms):
                  if state == LOW:                 鈥?Still pressed: confirmed
                    keyChar = KEY_MAP[row][col]
                    xQueueSend(queue_keyHandle, &keyChar, 0)
                    keyState = 2                   鈥?Mark as held
                  else:
                    keyState = 0                   鈥?False trigger: reset
              CASE 2 (held):
                if state == HIGH:                  鈥?Key released
                  keyState = 0                     鈥?Reset to idle

Line 95:      HAL_GPIO_WritePin(ROWS[row], ROWS[row].pin, SET)
              鈥?Restore row to HIGH (inactive)
Line 96:      taskYIELD()                           鈥?Yield to higher-priority tasks

Line 98:      vTaskDelay(pdMS_TO_TICKS(KEY_SCAN_INTERVAL_MS)) 鈥?5ms scan interval
Line 99:    } // end for(;;)

Line 103-106: void Task_APP_test(void *argument):
              vTaskDelete(NULL)                    鈥?Self-deleting test task

Line 108-111: void key_callback1():
              xTaskCreate(Task_APP_test, "APP", 256, NULL, osPriorityNormal2, NULL)
              鈥?Creates a temporary test task
```

**Keyboard Scan Timing:**
- Rows activated sequentially: row 0鈫?鈫?鈫?
- Per row: 1ms settling + 4 column reads + 鈮?.1ms GPIO operations
- Per cycle: ~4 脳 (1 + 0.5) = ~6ms actual, plus 5ms delay = ~11ms total
- Key detection: 20ms debounce (about 2 scan cycles)

---

## 12. Storage Layer

### 12.1 w25q256.h / w25q256.c 鈥?QSPI Flash Driver

**Header:** `Core/Inc/w25q256.h` (42 lines) | **Source:** `Core/Src/w25q256.c` (251 lines)

#### w25q256.h

```
Line 1-14:  Include guard + extern "C" + main.h + quadspi.h + stdbool.h

Line 16-22: Flash geometry constants:
Line 16:    #define W25Q256_SECTOR_SIZE      4096U    鈥?4 KB sector (erase unit)
Line 17:    #define W25Q256_BLOCK_SIZE        65536U  鈥?64 KB block (large erase unit)
Line 18:    #define W25Q256_PAGE_SIZE         256U    鈥?256 B page (write unit)
Line 19:    #define W25Q256_CAPACITY          33554432UL 鈥?32 MB total
Line 20:    #define W25Q256_SECTOR_COUNT      (33554432/4096) = 8192 sectors
Line 21:    #define W25Q256_MANUFACTURER_ID   0xEF    鈥?Winbond JEDEC ID
Line 22:    #define W25Q256_DEVICE_ID         0x4019  鈥?W25Q256JV device ID

Line 24-36: Public API:
Line 24:    bool W25Q256_Init(void)           鈥?Reset + detect + verify chip
Line 25:    bool W25Q256_Reset(void)          鈥?Enable Reset (0x66) + Reset (0x99)
Line 26:    uint32_t W25Q256_ReadID(void)     鈥?Read 3-byte JEDEC ID (0x9F command)
Line 27:    bool W25Q256_IsPresent(void)      鈥?Chip detected flag
Line 28:    uint32_t W25Q256_GetCapacity(void) 鈥?Return W25Q256_CAPACITY
Line 29:    const char* W25Q256_GetName(void)  鈥?Return "W25Q256 (32 MB)"
Line 31-32: bool W25Q256_EraseSector / EraseBlock (4KB / 64KB erase)
Line 33:    bool W25Q256_ChipErase(void)      鈥?Full chip erase (up to 120 seconds!)
Line 35-36: bool W25Q256_Read / Write (addr, buf, len) 鈥?Standard R/W operations
```

#### w25q256.c 鈥?Key Functions

```
Line 9-10:  Includes: w25q256.h
Line 11:    static bool g_w25q256_present = false 鈥?Chip detection state

Line 13-14: Static helpers: WriteEnable, WaitBusy

Line 16-30: bool W25Q256_Init(void):
Line 18:    W25Q256_Reset()                      鈥?Hardware reset: 0x66 + 0x99
Line 19:    uint32_t id = W25Q256_ReadID()       鈥?Read JEDEC ID (0x9F)
Line 20:    uint16_t dev_id = id & 0xFFFF        鈥?Extract device ID portion
Line 22-29: if dev_id == 0x4019:
              g_w25q256_present = true; return true
            else:
              g_w25q256_present = false; return false

Line 32-50: bool W25Q256_Reset(void):
Line 36-41: Send Enable Reset (0x66) via QSPI command
Line 44-45: Send Reset (0x99) via QSPI command
Line 48:    HAL_Delay(50)                        鈥?Wait for reset completion

Line 52-71: uint32_t W25Q256_ReadID(void):
Line 58:    sCmd.Instruction = 0x9F              鈥?JEDEC Read ID command
Line 59:    sCmd.AddressMode = QSPI_ADDRESS_NONE 鈥?No address, command only
Line 60:    sCmd.DataMode = QSPI_DATA_1_LINE     鈥?Single-line data read
Line 62:    sCmd.NbData = 3                      鈥?Read 3 bytes
Line 64-70: HAL_QSPI_Command + HAL_QSPI_Receive 鈫?rx[0..2]
              Return: (rx[0]<<16) | (rx[1]<<8) | rx[2]
              Expected: 0xEF4019 (Winbond W25Q256)

Line 88-107: bool W25Q256_EraseSector(uint32_t addr):
Line 96:    sCmd.Instruction = 0x20              鈥?Sector Erase command
Line 98-99: Address 24-bit, aligned to 4KB boundary
Line 106:   return W25Q256_WaitBusy(500)         鈥?Up to 500ms timeout

Line 109-128: bool W25Q256_EraseBlock(uint32_t addr):
Line 117:   sCmd.Instruction = 0xD8              鈥?Block Erase (64KB) command
Line 127:   return W25Q256_WaitBusy(3000)        鈥?Up to 3s timeout

Line 130-147: bool W25Q256_ChipErase(void):
Line 138:   sCmd.Instruction = 0xC7              鈥?Chip Erase command
Line 146:   return W25Q256_WaitBusy(120000)      鈥?Up to 120s timeout!

Line 149-168: bool W25Q256_Read(uint32_t addr, uint8_t *buf, uint32_t len):
Line 156:   sCmd.Instruction = 0x03              鈥?Standard Read command
Line 158-159: 24-bit address
Line 167:   HAL_QSPI_Receive(&hqspi, buf, ...)   鈥?Read data into buffer

Line 170-207: bool W25Q256_Write(uint32_t addr, const uint8_t *buf, uint32_t len):
              Page-aware write loop:
Line 178-204: while (remaining > 0):
Line 181:     chunk = min(remaining, W25Q256_PAGE_SIZE) 鈥?Max 256B per write
Line 183:     W25Q256_WriteEnable()                鈥?0x06 command
Line 186:     sCmd.Instruction = 0x02              鈥?Page Program command
Line 192:     sCmd.NbData = chunk
Line 194-197: HAL_QSPI_Command + HAL_QSPI_Transmit 鈥?Send data
Line 200:     W25Q256_WaitBusy(10)                 鈥?Wait for write completion
Line 202-203: remaining -= chunk; offset += chunk

Line 209-220: static bool W25Q256_WriteEnable(void):
Line 214:   sCmd.Instruction = 0x06               鈥?Write Enable command
              Must be called before each erase/program operation

Line 222-251: static bool W25Q256_WaitBusy(uint32_t timeout_ms):
Line 229:   sCmd.Instruction = 0x05               鈥?Read Status Register 1
Line 233:   sCmd.NbData = 1                       鈥?Read 1 byte
Line 235-248: do { ... } while ((HAL_GetTick() - start) < timeout):
              Read SR1 via QSPI
              Check bit 0 (BUSY): if 0 鈫?ready, return true
              HAL_Delay(1)                        鈥?1ms polling interval
```

### 12.2 lfs_port.h / lfs_port.c 鈥?LittleFS Port

**Header:** `Core/Inc/lfs_port.h` (22 lines) | **Source:** `Core/Src/lfs_port.c` (82 lines)

#### lfs_port.h

```
Line 1-8:   Include guard + extern "C"
Line 12:    #include "lfs.h"                      鈥?LittleFS core types
Line 14:    extern const struct lfs_config g_lfs_cfg 鈥?LFS configuration struct
Line 16:    int lfs_port_init(void)               鈥?Port initialization (returns 0)
```

#### lfs_port.c

```
Line 1-9:   File header: uses first 16 MB of W25Q256 for LittleFS
              Remaining 16 MB reserved or unused
Line 7:     #include "lfs_port.h"
Line 8:     #include "w25q256.h"

Line 10-13: LFS geometry:
Line 10:    #define LFS_READ_SIZE   256U            鈥?Read cache size
Line 11:    #define LFS_PROG_SIZE   256U            鈥?Program (write) size = page size
Line 12:    #define LFS_BLOCK_SIZE  4096U           鈥?Erase block = 4 KB (matches flash sector)
Line 13:    #define LFS_BLOCK_COUNT 4096U           鈥?4096 blocks 脳 4KB = 16 MB data area

Line 15-20: Static callback forward declarations:
Line 15-16: lfs_read 鈥?Read from flash
Line 17-18: lfs_prog 鈥?Program (write) to flash
Line 19:    lfs_erase 鈥?Erase flash block
Line 20:    lfs_sync 鈥?Sync operation (no-op for bare-metal flash)

Line 22-24: Static buffers:
Line 22:    static uint8_t g_lfs_read_buf[256]     鈥?LFS read cache buffer
Line 23:    static uint8_t g_lfs_prog_buf[256]     鈥?LFS program cache buffer
Line 24:    static uint8_t g_lfs_lookahead_buf[32] 鈥?LFS block allocator lookahead

Line 26-42: const struct lfs_config g_lfs_cfg = {
Line 28-31: .read /.prog /.erase /.sync          鈥?Callback function pointers
Line 32:    .read_size      = 256                  鈥?Minimum read unit
Line 33:    .prog_size      = 256                  鈥?Minimum write unit = page size
Line 34:    .block_size     = 4096                 鈥?Erase block = sector size
Line 35:    .block_count    = 4096                 鈥?16 MB / 4 KB
Line 36:    .cache_size     = 256                  鈥?File cache size
Line 37:    .lookahead_size = sizeof(g_lfs_lookahead_buf) = 32
Line 38:    .block_cycles   = 500                  鈥?Wear-leveling block cycle count
Line 39-41: .read_buffer / .prog_buffer / .lookahead_buffer 鈥?Pre-allocated buffers

Line 44-47: int lfs_port_init(void): return 0      鈥?No init needed

--- Block Operation Callbacks (lines 49-82) ---

Line 49-57: static int lfs_read(c, block, off, buf, size):
Line 53:    uint32_t addr = block 脳 LFS_BLOCK_SIZE + off
              鈥?Calculate absolute flash address from block number and offset
Line 54:    if (W25Q256_Read(addr, buf, size)) return 0   鈥?Success
            else                              return LFS_ERR_IO 鈥?Error

Line 59-67: static int lfs_prog(c, block, off, buf, size):
Line 63-66: Same address calculation 鈫?W25Q256_Write 鈫?0 or LFS_ERR_IO

Line 69-76: static int lfs_erase(c, block):
Line 72-75: W25Q256_EraseSector(block 脳 4096) 鈫?0 or LFS_ERR_IO
              鈥?Erases one 4KB sector (exactly LFS block size)

Line 78-82: static int lfs_sync(c): return 0       鈥?No-op (no OS caching layer)
```

**LittleFS on W25Q256 Summary:**
- Total flash: 32 MB
- LFS partition: 0鈥?6 MB (4096 blocks 脳 4 KB)
- Page size: 256 bytes
- Block size: 4096 bytes (1 sector)
- Wear leveling: 500 cycles per block
- All metadata in flash (no external config storage needed)

---

## 13. Interrupt Handlers

### 13.1 stm32h7xx_it.c 鈥?ISR Implementation

**Path:** `Core/Src/stm32h7xx_it.c` | **Lines:** 1-253 | **USER CODE sections only**

```
Line 1-18:  STM CubeMX copyright header
Line 21-23: Includes: main.h, stm32h7xx_it.h

Line 24-26: USER CODE BEGIN Includes:
Line 25:    #include "PWM.h"                      鈥?Access to dutyA/B/C/D, pwm_counter

Line 28-32: USER CODE BEGIN TD: (Typedef)
Line 30:    extern uint16_t pwm_counter           鈥?PWM period counter
Line 31:    extern uint16_t dutyA, dutyB, dutyC, dutyD 鈥?PWM duty thresholds
              鈥?Declared extern; defined in PWM.c

Line 59-64: extern variables for peripheral handles:
Line 60:    extern PCD_HandleTypeDef hpcd_USB_OTG_FS 鈥?USB OTG FS handle
Line 61:    extern TIM_HandleTypeDef htim6           鈥?TIM6 (SW PWM + timebase)
Line 62:    extern UART_HandleTypeDef huart4          鈥?UART4 handle
Line 63:    extern UART_HandleTypeDef huart3          鈥?USART3 handle
Line 64:    extern TIM_HandleTypeDef htim7           鈥?TIM7 (HAL SysTick timebase)

--- Fault Handlers (lines 76-160) ---
Line 76-87:  NMI_Handler: HAL_RCC_NMI_IRQHandler() + while(1) loop
              鈥?Handles Clock Security System NMI
Line 92-102: HardFault_Handler: while(1) {...}     鈥?Hard fault: halt
Line 107-117: MemManage_Handler: while(1) {...}     鈥?Memory fault: halt
Line 122-132: BusFault_Handler: while(1) {...}      鈥?Bus fault: halt
Line 137-147: UsageFault_Handler: while(1) {...}    鈥?Usage fault: halt
Line 152-160: DebugMon_Handler: (empty)             鈥?Debug monitor exception

--- Peripheral ISRs (lines 169-249) ---
Line 172-181: USART3_IRQHandler:
              HAL_UART_IRQHandler(&huart3)          鈥?Standard HAL UART ISR
              USER CODE sections: (empty)

Line 186-195: UART4_IRQHandler:
              HAL_UART_IRQHandler(&huart4)          鈥?Standard HAL UART ISR

Line 200-221: TIM6_DAC_IRQHandler 鈥?SOFTWARE PWM IMPLEMENTATION:
Line 203:   pwm_counter--;                          鈥?Decrement PWM counter
Line 204:   if (pwm_counter == 0) {                 鈥?Period rollover (20ms cycle)
Line 205:     pwm_counter = 500;                    鈥?Reset to 500 (PWM period)
Line 206:   }
Line 209:   HAL_TIM_IRQHandler(&htim6)              鈥?Standard HAL timer ISR
Line 211-218: PWM output comparison:
Line 211:   if (pwm_counter < dutyA) 鈫?PWMA_Pin = SET   else 鈫?RESET
Line 213:   if (pwm_counter < dutyB) 鈫?PWMB_Pin = SET   else 鈫?RESET
Line 215:   if (pwm_counter < dutyC) 鈫?PWMC_Pin = SET   else 鈫?RESET
Line 217:   if (pwm_counter < dutyD) 鈫?PWMD_Pin = SET   else 鈫?RESET
              鈥?4-channel software PWM: counter counts down 500鈫?,
                duty values 0-500 determine ON time.
                PWM frequency = TIM6 period / 500

TIM6 Configuration for PWM:
  Counter period: TIM6->ARR = ?
  PWM resolution: 500 steps 鈫?0.2% duty cycle resolution
  PWM frequency: depends on TIM6 prescaler
  If TIM6 counter clock = 1 MHz 鈫?PWM freq = 1MHz/500 = 2 kHz 鈫?not 50 Hz
  If TIM6 counter clock = 25 kHz 鈫?PWM freq = 25kHz/500 = 50 Hz (standard servo)
  Implementation note: pwm_counter decremented per ISR, so ISR fire rate = PWM freq 脳 500

Line 226-235: TIM7_IRQHandler:
              HAL_TIM_IRQHandler(&htim7)            鈥?HAL timebase (TIM7)
              Callback: HAL_TIM_PeriodElapsedCallback 鈫?HAL_IncTick()

Line 240-249: OTG_FS_IRQHandler (USB):
              HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS)  鈥?USB device interrupt
```

### 13.2 stm32h7xx_hal_msp.c 鈥?MSP Initialization

**Path:** `Core/Src/stm32h7xx_hal_msp.c` | **Lines:** 1-82 | **USER CODE sections only**

```
Line 1-18:  STM CubeMX copyright header
Line 21-22: Includes: main.h
Line 23-24: USER CODE BEGIN Includes: (empty)
              All USER CODE sections in this file are empty (no custom MSP code)

Line 62-78: void HAL_MspInit(void):
Line 69:    __HAL_RCC_SYSCFG_CLK_ENABLE()          鈥?Enable system config clock
              Required for MPU and EXTI configuration
Line 73:    HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0)
              鈥?Set PendSV to lowest priority (15 sub-priority 0)
              鈥?PendSV is used for FreeRTOS context switching

              NVIC priority grouping: 4 bits preempt, 0 bits sub
              PendSV priority = 15 (lowest possible)
              This ensures PendSV only runs when no other interrupt is active.
```

---

## 14. App Thin-Header Wrappers

All app_*.h headers follow the same pattern 鈥?thin wrappers that only `#include "app.h"`:

| Header File | Lines | Purpose |
|-------------|-------|---------|
| `app_init.h` | 8 | One-shot init task |
| `app_common.h` | 4 | Common app definitions |
| `app_control.h` | 4 | Car control service |
| `app_sensor.h` | 4 | IMU sensor service |
| `app_device.h` | 4 | Display/broadcast/UART/keyboard |
| `app_sysmon.h` | 8 | System monitor |
| `app_console.h` | 8 | CLI console |
| `app_qspi.h` | 8 | QSPI flash service |
| `app_rtos_hooks.h` | 8 | FreeRTOS hooks |
| `app_robot.h` | 4 | Robotic arm |
| `display_service.h` | 8 | Display abstraction layer |

This pattern ensures each `.c` file can include its own header while all CubeMX files include only `app.h`.

---

## 15. Complete FreeRTOS Task Table

### CubeMX-Managed Tasks (osThreadNew)

| Task Name | Handler | Stack (B) | Priority | Function | Period |
|-----------|---------|-----------|----------|----------|--------|
| LEDBlink | LedBlink_Handler | 512 | AboveNormal | USB init + LED 1Hz toggle | 1 s |
| Broadcast | Broadcast_Handler | 1024 | Normal1 | BT broadcast (disabled) | 50 ms |
| Uart4Rx | Uart4Rx_Handler | 1024 | Normal7 | UART4 frame RX | Blocking |
| IMUService | IMUService_Handler | 512 | Normal3 | IMU AHRS update | 5 ms |
| CarControl | CarControl_Handler | 512 | Normal5 | 7-step control loop | 10 ms |
| KeyScan | KeyScan_Handler | 512 | Normal6 | Keyboard scan (stub) | 1 ms |
| Buzzer | Buzzer_Handler | 512 | Normal1 | Music player (disabled) | 1 ms |
| Display | Display_Handler | 512 | Normal1 | Display update | 100 ms |
| Reserved | Reserved_Handler | 4096 | Normal1 | Future placeholder | 50 ms |
| Uart3Rx | Uart3Rx_Handler | 1024 | Normal7 | UART3 frame RX | Blocking |
| SysMon | SysMon_Handler | 2048 | Low | Heap/stack monitor | 200 ms |
| Console | Console_Handler | 2048 | Low | CLI console | Blocking |

### Dynamically Created Tasks (xTaskCreate)

| Task Name | Function | Stack (B) | Priority | Function | Period |
|-----------|----------|-----------|----------|----------|--------|
| AppInit | AppInit_Task | 2048 | Normal5 | USB init + car cmd | One-shot |
| KeyScan | vKeyScanTask | 1024 | Idle+2 | Matrix scan + debounce | ~11 ms |

### FreeRTOS Kernel Tasks

| Task Name | Stack (B) | Priority | Function |
|-----------|-----------|----------|----------|
| IDLE | 128 | 0 | Idle task (runs when nothing else ready) |
| Tmr Svc | 1024 | 55 | Software timer service (highest priority) |

### Priority Ladder (56 = Highest)

```
55: Tmr Svc (FreeRTOS software timers)
...
7:  AboveNormal 鈥?LEDBlink
6:  Normal7 鈥?Uart3Rx, Uart4Rx (UART data critical)
5:  Normal6 鈥?KeyScan (CubeMX)
4:  Normal5 鈥?CarControl, AppInit
3:  Normal3 鈥?IMUService
2:  Idle+2 鈥?vKeyScanTask (FreeRTOS)
1:  Normal1 鈥?Broadcast, Buzzer, Display, Reserved
0:  Low 鈥?SysMon, Console
-:  IDLE (priority 0, lowest)
```

**Total Task Stack Allocation (estimate):**

| Category | Stack |
|----------|-------|
| CubeMX tasks (12) | 16,896 B |
| Dynamic tasks (AppInit + KeyScan) | 3,072 B |
| Kernel tasks (IDLE + Tmr) | 1,152 B |
| **Total static stack** | **~21 KB** |
| **Heap (configTOTAL_HEAP_SIZE)** | **80 KB** |

---

## 16. Interrupt Vector Table

### Exception Handlers

| Exception | IRQn | Handler | Priority | Notes |
|-----------|------|---------|----------|-------|
| Reset | -15 | Reset_Handler | 鈥?| Startup code |
| NMI | -14 | NMI_Handler | -2 | Clock Security System |
| HardFault | -13 | HardFault_Handler | -1 | Halt forever |
| MemManage | -12 | MemManage_Handler | Config | Halt forever |
| BusFault | -11 | BusFault_Handler | Config | Halt forever |
| UsageFault | -10 | UsageFault_Handler | Config | Halt forever |
| SVCall | -5 | SVC_Handler | 鈥?| FreeRTOS SVC for context switch |
| DebugMon | -4 | DebugMon_Handler | Config | (empty) |
| PendSV | -2 | PendSV_Handler | 15 | FreeRTOS context switch (lowest) |
| SysTick | -1 | SysTick_Handler | 鈥?| FreeRTOS tick **OR** custom |

### Peripheral Interrupts

| Peripheral | IRQ | Handler | Priority | User Code Actions |
|-----------|-----|---------|----------|-------------------|
| TIM6 + DAC | 54 | TIM6_DAC_IRQHandler | Config | **SW PWM**: pwm_counter--, GPIO compare with dutyA/B/C/D |
| TIM7 | 55 | TIM7_IRQHandler | Config | HAL_IncTick() via callback (1ms timebase) |
| USART3 | 39 | USART3_IRQHandler | Config | HAL_UART_IRQHandler 鈫?uart_fifo.c callback |
| UART4 | 52 | UART4_IRQHandler | Config | HAL_UART_IRQHandler 鈫?uart_fifo.c callback |
| OTG_FS | 67 | OTG_FS_IRQHandler | Config | HAL_PCD_IRQHandler 鈫?USB CDC data |
| TIM2 (LF encoder) | 28 | TIM2_IRQHandler | Config | CubeMX-generated encoder handling |
| TIM3 (LR encoder) | 29 | TIM3_IRQHandler | Config | CubeMX-generated encoder handling |
| TIM4 (RF encoder) | 30 | TIM4_IRQHandler | Config | CubeMX-generated encoder handling |
| TIM5 (RR encoder) | 31 | TIM5_IRQHandler | Config | CubeMX-generated encoder handling |

### Interrupt Priority Rules

- **configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5**
  Interrupts with priority 鈮?5 (lower numeric value) **CANNOT** use FreeRTOS ISR-safe API functions.
  Practically: priority 6-15 may use xQueueSendFromISR, etc.

- **configKERNEL_INTERRUPT_PRIORITY = 15** (lowest)
  PendSV must be lowest to avoid interrupting other ISRs.

- **TIM6 SW PWM ISR**: Must be high enough priority to maintain accurate PWM timing
  but must NOT call FreeRTOS ISR functions if priority > 5 (numeric < 5).

---

## 17. Pin Map

### Motor Control

| Function | Pin | Port | TIM | Channel | Description |
|----------|-----|------|-----|---------|-------------|
| LFM_IN1 | PE3 | GPIOE | 鈥?| 鈥?| Left Front Motor direction 1 |
| LFM_IN2 | PC13 | GPIOC | 鈥?| 鈥?| Left Front Motor direction 2 |
| LRM_IN1 | PC3 | GPIOC | 鈥?| 鈥?| Left Rear Motor direction 1 |
| LRM_IN2 | PC0 | GPIOC | 鈥?| 鈥?| Left Rear Motor direction 2 |
| RFM_IN1 | PB1 | GPIOB | 鈥?| 鈥?| Right Front Motor direction 1 |
| RFM_IN2 | PE7 | GPIOE | 鈥?| 鈥?| Right Front Motor direction 2 |
| RRM_IN1 | PC5 | GPIOC | 鈥?| 鈥?| Right Rear Motor direction 1 |
| RRM_IN2 | PC4 | GPIOC | 鈥?| 鈥?| Right Rear Motor direction 2 |
| PWMA | ? | ? | 鈥?| 鈥?| Software PWM Channel A (LF) |
| PWMB | ? | ? | 鈥?| 鈥?| Software PWM Channel B (LR) |
| PWMC | ? | ? | 鈥?| 鈥?| Software PWM Channel C (RR) |
| PWMD | ? | ? | 鈥?| 鈥?| Software PWM Channel D (RF) |

### Encoders (TIM2-5 in encoder mode)

| Function | TIM | Channel | Description |
|----------|-----|---------|-------------|
| ENC_LF | TIM2 | CH1/CH2 | Left Front encoder (quadrature) |
| ENC_LR | TIM3 | CH1/CH2 | Left Rear encoder (quadrature) |
| ENC_RF | TIM4 | CH1/CH2 | Right Front encoder (quadrature) |
| ENC_RR | TIM5 | CH1/CH2 | Right Rear encoder (quadrature) |

### Communication

| Peripheral | RX Pin | TX Pin | Function |
|-----------|--------|--------|----------|
| USART1 | PA10 | PA9 | Debug/legacy UART |
| USART2 | PD6 | PD5 | FT Servo control |
| USART3 | PD9 | PD8 | UART3 frame RX ("cat") |
| UART4 | PD1 | PD0 | BT broadcast / UART4 frame |
| UART5 | PB12 | PB13 | Spare UART |
| SPI2 | 鈥?| 鈥?| OLED display (SPI) |
| SPI4 | 鈥?| 鈥?| ICM-45686 IMU sensor (SPI) |
| QSPI | 鈥?| 鈥?| W25Q256 NOR Flash |
| USB_OTG_FS | 鈥?| 鈥?| USB CDC Virtual COM Port |

### Keyboard Matrix (4脳4)

| Row | Pin | Port |
|-----|-----|------|
| ROW1 | PE9 | GPIOE |
| ROW2 | PE11 | GPIOE |
| ROW3 | PE13 | GPIOE |
| ROW4 | PE14 | GPIOE |

| Col | Pin | Port |
|-----|-----|------|
| COL1 | PA10 | GPIOA |
| COL2 | PB5 | GPIOB |
| COL3 | PC9 | GPIOC |
| COL4 | PC8 | GPIOC |

### IMU (ICM-45686 via SPI4)

| Signal | Pin | Description |
|--------|-----|-------------|
| SPI4_SCK | PE2 | SPI clock |
| SPI4_MISO | PE5 | Master In Slave Out |
| SPI4_MOSI | PE6 | Master Out Slave In |
| IMU_CS | ? | Chip select |
| IMU_INT | ? | Data ready interrupt |

### Display (OLED via SPI2)

| Signal | Pin | Description |
|--------|-----|-------------|
| SPI2_SCK | PB13 | SPI clock |
| SPI2_MOSI | PC1 | Master Out Slave In |
| OLED_DC | PC6 | Data/Command select |
| OLED_RES | PC8 | Reset |
| OLED_CS | 鈥?| Chip select |

### Miscellaneous

| Function | Pin | Port | Description |
|----------|-----|------|-------------|
| LED0 | ? | ? | Heartbeat LED |
| LED2 | ? | ? | Debug LED (used in uart_fifo) |

---

## 18. Memory Layout

### STM32H750VBT6 Memory

| Region | Size | Address | Usage |
|--------|------|---------|-------|
| FLASH | 128 KB | 0x08000000 | Application code + constant data |
| DTCM | 128 KB | 0x20000000 | Critical data (stack, TCB) |
| AXI SRAM | 512 KB | 0x24000000 | Main SRAM (heap, buffers) |
| SRAM1 | 128 KB | 0x30000000 | General purpose |
| SRAM2 | 128 KB | 0x30020000 | General purpose |
| SRAM3 | 32 KB | 0x30040000 | General purpose |
| SRAM4 | 64 KB | 0x38000000 | General purpose |
| ITCM | 64 KB | 0x00000000 | Fast instruction RAM |

### Memory Allocation Estimate

| Component | Memory | Size |
|-----------|--------|------|
| FreeRTOS heap (heap_4) | AXI SRAM | 80 KB |
| Task stacks (static) | DTCM/AXI | ~21 KB |
| Task TCBs (static) | DTCM | ~1 KB |
| Queue buffers | AXI SRAM | ~1 KB |
| Stream buffer (console) | AXI SRAM | 256 B |
| LFS buffers (read/prog/lookahead) | AXI SRAM | 544 B |
| UART frame buffers | AXI SRAM | ~500 B |
| IMU ring buffer (300脳3 doubles) | AXI SRAM | ~7.2 KB |
| OLED framebuffer | AXI SRAM | ~1 KB |
| FreeRTOS kernel structures | DTCM | ~5 KB |
| **Estimated total** | 鈥?| **~116 KB** |

### QSPI Flash (W25Q256, 32 MB)

| Region | Size | Usage |
|--------|------|-------|
| LittleFS partition | 16 MB | File system (configs, logs, data) |
| Reserved / spare | 16 MB | Firmware update images, backup |

---

## 19. Build Instructions

### Prerequisites

1. STM32CubeMX 6.x+ with STM32H7 package
2. MDK-ARM (Keil) V5.32 or newer
3. ARM Compiler 5 or 6

### Build Steps

1. **Open CubeMX project** (`.ioc` file)
2. **Generate code** (Project 鈫?Generate Code)
3. **Open MDK-ARM project** (`.uvprojx` file)
4. **Build** (Project 鈫?Build Target, F7)
5. **Flash** (Flash 鈫?Download, F8)

### Important Build Notes

1. `configCPU_CLOCK_HZ` must be `SystemCoreClock / 2U` (see FreeRTOSConfig.h line 182)
   SystemCoreClock = 480 MHz, HCLK = 240 MHz 鈫?SysTick clock = 240 MHz

2. FLASH latency must be `FLASH_LATENCY_4` for 480 MHz VOS0 operation

3. HEAP selection: `heap_4.c` (defined by `USE_FreeRTOS_HEAP_4`)

4. Static allocation preferred: all 12 CubeMX tasks use static TCBs and stacks

5. LittleFS compilation: `lfs.c` and `lfs_util.c` must be added to the project

6. FreeRTOS+CLI compilation: `FreeRTOS_CLI.c` must be added to the project

7. Required compiler preprocessor defines (typical for H750):
   - `USE_HAL_DRIVER`
   - `STM32H750xx`
   - `ARM_MATH_CM7` (if using DSP)

---

## 20. CubeMX Regeneration Checklist

When regenerating code from CubeMX, follow this checklist to restore hand-written code:

### Files to Restore / Merge

| File | Action | Notes |
|------|--------|-------|
| `Core/Inc/app.h` | Keep existing | Main facade 鈥?not generated by CubeMX |
| `Core/Inc/config.h` | Keep existing | Config aggregator 鈥?not generated by CubeMX |
| `Core/Inc/platform_config.h` | Keep existing | Platform constants 鈥?not generated |
| `Core/Inc/motor_config.h` | Keep existing | Motor PID 鈥?not generated |
| `Core/Inc/control_config.h` | Keep existing | Control PID 鈥?not generated |
| `Core/Inc/rtos_config.h` | Keep existing | RTOS timing 鈥?not generated |
| `Core/Inc/FreeRTOSConfig.h` | **Merge** | USER CODE sections must be preserved (lines 159-160, 172-183) |
| `Core/Src/main.c` | **Merge** | USER CODE sections: Includes, PD, PV, Init, 2, 4 |
| `Core/Src/freertos.c` | **Merge** | USER CODE sections: Includes, Variables, RTOS_QUEUES, RTOS_THREADS, ALL 12 Handler bodies |
| `Core/Src/stm32h7xx_it.c` | **Merge** | USER CODE Includes (PWM.h) + TIM6_DAC ISR body |
| `Core/Src/stm32h7xx_hal_msp.c` | Keep generated | No custom MSP code |

### File Preservation Strategy

All non-CubeMX files (listed below) are safe from regeneration:
- `Core/Src/app_*.c` (init, control, sensor, device, sysmon, console, qspi, rtos_hooks)
- `Core/Src/pid.c`, `PWM.c`, `Motor.c`, `car_attitude.c`, `car_control.c`
- `Core/Src/IMU.c`, `read_aux_data_mode.c`
- `Core/Src/delay.c`, `sys_time.c`
- `Core/Src/uart_fifo.c`, `keyboard.c`
- `Core/Src/w25q256.c`, `lfs_port.c`
- `Core/Src/display_service.c`
- All `Core/Inc/` headers except those in the merge list above

### Post-Regeneration Checklist

- [ ] Verify `configCPU_CLOCK_HZ = SystemCoreClock / 2U` in FreeRTOSConfig.h
- [ ] Verify all USER CODE sections are intact in freertos.c (12 handlers)
- [ ] Verify TIM6_DAC ISR has PWM generation code
- [ ] Verify `#include "app.h"` in main.c and freertos.c
- [ ] Verify CubeMX USB_DEVICE Platform Settings: initialization task = "none"
- [ ] Rebuild and test LED heartbeat
- [ ] Test CLI console via USB CDC
- [ ] Test car control loop (encoders + motors)
- [ ] Verify SysMon heap and stack display on OLED
- [ ] Verify LittleFS mount works on QSPI flash

### Common Issues After Regeneration

1. **`osKernelStart()` hangs**: CubeMX may auto-insert `MX_USB_DEVICE_Init()` in LedBlink_Handler.
   Solution: In CubeMX 鈫?USB_DEVICE 鈫?Platform Settings, set init task to "none".
   USB init is handled by `AppInit_Task`.

2. **SysTick at wrong rate**: If `configCPU_CLOCK_HZ` is not corrected, all `osDelay()` times are 2脳 too long.
   Verify line 182 in FreeRTOSConfig.h.

3. **Missing PWM**: If TIM6_DAC ISR code is lost, motors will not spin.
   Restore from backup: `pwm_counter--`, duty comparison, GPIO output.

4. **UART frame broken**: if uart_frame_rx_init() not called, UART3/4 receive queues remain empty.
   Ensure `uart_frame_rx_init()` is called somewhere in initialization.

5. **LittleFS link errors**: Ensure `lfs.c`, `lfs_util.c`, and `FreeRTOS_CLI.c` are in the project source list.

---

## 21. Module Communication Map

```
鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?鈹?                        FREERTOS SCHEDULER                          鈹?鈹溾攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?鈹?                                                                    鈹?鈹? 鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?   xQueue           鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹?鈹? 鈹?TIM6 ISR 鈹傗攢鈹€鈹€(dutyA/B/C/D)鈹€鈹€鈹€鈹€鈫掆攤  GPIO    鈹? Software PWM       鈹?鈹? 鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹?鈹?                                                                    鈹?鈹? 鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?   StreamBuffer     鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹?鈹? 鈹?USB ISR  鈹傗攢鈹€鈹€(CDC RX data)鈹€鈹€鈹€鈹€鈫掆攤 Console  鈹? CLI over USB       鈹?鈹? 鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹?鈹?                                                                    鈹?鈹? 鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹? xQueueSendFromISR  鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹?鈹? 鈹?UART3 ISR鈹傗攢鈹€鈹€(line frames)鈹€鈹€鈹€鈹€鈫掆攤 Uart3Rx  鈹? Serial frames      鈹?鈹? 鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹?鈹?                                                                    鈹?鈹? 鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹? xQueueSendFromISR  鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹?鈹? 鈹?UART4 ISR鈹傗攢鈹€鈹€(line frames)鈹€鈹€鈹€鈹€鈫掆攤 Uart4Rx  鈹? Serial frames      鈹?鈹? 鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹?鈹?                                                                    鈹?鈹? 鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹? xQueueSend         鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹?鈹? 鈹?KeyScan  鈹傗攢鈹€鈹€(char events)鈹€鈹€鈹€鈹€鈫掆攤 consumer 鈹? Key input          鈹?鈹? 鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹?鈹?                                                                    鈹?鈹? 鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹? global variables   鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹?鈹? 鈹?IMUServ. 鈹傗攢鈹€鈹€(imu_g_z,ypr)鈹€鈹€鈹€鈹€鈫掆攤 CarAttit 鈹? Angular velocity   鈹?鈹? 鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹?鈹?                                                                    鈹?鈹? 鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹? Wheel_Set_V_Real   鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹?鈹? 鈹?CarAttit 鈹傗攢鈹€鈹€(v_left,v_right)鈫掆攤  Motor   鈹? Speed targets       鈹?鈹? 鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹?鈹?                                                                    鈹?鈹? 鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹? Set_PWM_Duty       鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹?鈹? 鈹? Motor   鈹傗攢鈹€鈹€(channel,duty)鈹€鈹€鈹€鈫掆攤   PWM    鈹? Duty cycle set      鈹?鈹? 鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹?鈹?                                                                    鈹?鈹? 鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹? car_state_t        鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹?鈹? 鈹?CarAttit 鈹傗梽鈹€鈹€鈹€(yaw,ypr)鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈫掆攤 CarContr 鈹? Shared state       鈹?鈹? 鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹?鈹?                                                                    鈹?鈹? 鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹? lfs API            鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹?鈹? 鈹?Console  鈹傗攢鈹€鈹€(mount/ls/cat)鈹€鈹€鈹€鈫掆攤 AppQSPI  鈹? File system ops    鈹?鈹? 鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹?鈹?                                      鈹?                            鈹?鈹?                                 鈹屸攢鈹€鈹€鈹€鈹€鈹粹攢鈹€鈹€鈹€鈹€鈹?                    鈹?鈹?                                 鈹? lfs_port 鈹? Block ops          鈹?鈹?                                 鈹斺攢鈹€鈹€鈹€鈹€鈹攢鈹€鈹€鈹€鈹€鈹?                    鈹?鈹?                                 鈹屸攢鈹€鈹€鈹€鈹€鈹粹攢鈹€鈹€鈹€鈹€鈹?                    鈹?鈹?                                 鈹? W25Q256  鈹? QSPI HAL           鈹?鈹?                                 鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹?鈹?                                                                    鈹?鈹? 鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹? DisplayService     鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹?鈹? 鈹? SysMon  鈹傗攢鈹€鈹€(ShowText)鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈫掆攤 Display  鈹? OLED output        鈹?鈹? 鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                    鈹?鈹?                                                                    鈹?鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?```

---

## 22. Data Flow: Complete Control Cycle

```
[IMUService Task 鈥?5ms]
    鈹?    鈹溾攢 ICM-45686 鈫?SPI4 鈫?inv_imu_driver 鈫?bsp_IcmGetRawData()
    鈹?  鈫?IMU_getValues() 鈫?IMU_AHRSupdate()
    鈹?  鈫?quaternion 鈫?IMU_getYawPitchRoll(ypr)
    鈹?  鈫?imu_g_z (Z-axis gyro)
    鈹?    鈹斺攢 Car_Attitude_Update_Input()
        鈫?car_state.yaw updated via Car_Attitude_Yaw_Update()

[CarControl Task 鈥?10ms]
    鈹?    鈹溾攢 Tim_Update_Enc_Speed()          鈫?Encoder pulse deltas from TIM2-5
    鈹溾攢 Motor_Update_Input_All()        鈫?Pulses 鈫?v_enc 鈫?v_real (mm/s)
    鈹?  鈹斺攢 Motor_Judge_Accuracy()      鈫?Select best encoder for wheel_left/right
    鈹溾攢 Car_Attitude_Update_Input()     鈫?v_line, v_angle from wheels + IMU
    鈹溾攢 Car_Control_Update_Input()      鈫?Progress check (distance/angle)
    鈹?  鈹斺攢 If done 鈫?oprate_done = 1 鈫?STOP
    鈹溾攢 Car_Control_Update_Output()     鈫?Position PID 鈫?target_v_line/v_angle
    鈹?  鈹斺攢 Set_Car_Attitude(v_line, v_angle)
    鈹溾攢 Car_Attitude_Update_Output()    鈫?Angular PID 鈫?v_left, v_right
    鈹?  鈹斺攢 Wheel_Set_V_Real(v_left, v_right)
    鈹斺攢 Motor_Update_Output_All()       鈫?Speed PID 鈫?PWM duty
        鈹斺攢 Motor_Forward/Backward 鈫?GPIO direction
        鈹斺攢 Motor_SetPWM 鈫?Set_PWM_Duty 鈫?dutyA/B/C/D

[TIM6 ISR 鈥?~50 Hz 脳 500 steps]
    鈹?    鈹斺攢 pwm_counter-- (500 鈫?0)
       if pwm_counter < dutyX 鈫?GPIO SET (motor ON)
       else                   鈫?GPIO RESET (motor OFF)
```

---

*Version: v3.1 | Updated: 2026-05-09*
*Documentation covers all 45 source files across 6 architectural layers.*


---

## 23. Complete Variable Reference

### Global Variables (defined across the codebase)

| Variable | Type | File:Line | Scope | Description |
|----------|------|-----------|-------|-------------|
| `motion6[7]` | float[7] | main.c:59 | extern in app.h | Raw IMU: accel[3]+gyro[3]+temp |
| `ypr[3]` | float[3] | main.c:60 | extern in app.h | Yaw[0], Pitch[1], Roll[2] in degrees |
| `math_pl` | int | main.c:61 | extern in app.h | Math placeholder |
| `cnt` | uint8_t | main.c:58 | local (static) | Unused counter |
| `dutyA` | volatile uint16_t | PWM.c:8 | extern in app.h | PWM Ch0 (LF) duty 0-500 |
| `dutyB` | volatile uint16_t | PWM.c:8 | extern in app.h | PWM Ch1 (LR) duty 0-500 |
| `dutyC` | volatile uint16_t | PWM.c:8 | extern in app.h | PWM Ch2 (RR) duty 0-500 |
| `dutyD` | volatile uint16_t | PWM.c:8 | extern in app.h | PWM Ch3 (RF) duty 0-500 |
| `pwm_counter` | volatile uint16_t | PWM.c:7 | extern in app.h | 500-count PWM period counter |
| `motor_LeftFront` | motor | Motor.c:8 | extern in Motor.h | LF motor struct |
| `motor_LeftRear` | motor | Motor.c:9 | extern in Motor.h | LR motor struct |
| `motor_RightFront` | motor | Motor.c:10 | extern in Motor.h | RF motor struct |
| `motor_RightRear` | motor | Motor.c:11 | extern in Motor.h | RR motor struct |
| `wheel_left` | motor* | Motor.c:15 | static | Best left-side encoder ref |
| `wheel_right` | motor* | Motor.c:16 | static | Best right-side encoder ref |
| `car_attitude` | _car_attitude | car_attitude.c:18 | extern | Attitude PID + speed targets |
| `car_state` | car_state_t | car_attitude.c:19 | extern in app.h | Shared yaw/speed state |
| `car_control` | _car_control | car_control.c:7 | extern | Motion control state machine |
| `q0,q1,q2,q3` | volatile float | IMU.c:21 | local (static) | Attitude quaternion |
| `exInt,eyInt,ezInt` | volatile float | IMU.c:20 | local (static) | Gyro bias error integrals |
| `lastUpdate, now` | volatile uint32_t | IMU.c:23 | local (static) | AHRS time tracking (us) |
| `yaw[5]` | volatile float[5] | IMU.c:24 | extern in IMU.h | Yaw ring buffer |
| `imu_g_z` | float | IMU.c:28 | extern in app.h | Z-axis gyro angular rate |
| `gyro_offset[3]` | float[3] | IMU.c:104 | local (static) | Auto-calibrated gyro offsets |
| `TTangles_gyro[7]` | float[7] | IMU.c:26 | local (static) | Raw sensor data buffer |
| `nowtime` | uint32_t | sys_time.c:9 | extern in sys_time.h | System time (100us units) |
| `uart3_frame_queue` | QueueHandle_t | freertos.c:53 | extern in app.h | UART3 5-slot frame queue |
| `uart4_frame_queue` | QueueHandle_t | freertos.c:54 | extern in app.h | UART4 5-slot frame queue |
| `console_rx_stream` | StreamBufferHandle_t | freertos.c:56 | extern in app.h | USB CDC RX stream (256B) |
| `queue_keyHandle` | osMessageQueueId_t | freertos.c:205 | extern in keyboard.h | Key press char queue |
| `semphr_buzzer_triggerHandle` | osSemaphoreId_t | freertos.c:210 | local | Buzzer binary semaphore |
| `semphr_uart_receiveHandle` | osSemaphoreId_t | freertos.c:215 | local | UART counting semaphore |
| `xMotionTaskHandle` | TaskHandle_t | freertos.c:55 | local | Unused motion task handle |
| `uart_rx_byte` | uint8_t | uart_fifo.c:34 | extern in uart_fifo.h | USART1 RX byte buffer |
| `uart_received_frame` | uint8_t[MAX_FRAME_SIZE*8] | uart_fifo.c:37 | extern | USART1 bit-formatted frame |
| `uart_received_frame_length` | uint16_t | uart_fifo.c:38 | extern | Frame bit length |
| `uart_tx_buffer` | uint8_t[MAX_FRAME_SIZE+2] | uart_fifo.c:39 | extern | USART1 TX frame buffer |
| `uart_tx_length` | uint16_t | uart_fifo.c:40 | extern | TX buffer byte length |
| `g_lfs` | lfs_t | app_qspi.c:21 | static | LittleFS filesystem instance |
| `g_lfs_mounted` | bool | app_qspi.c:22 | static | LFS mount state |
| `g_lfs_cfg` | lfs_config | lfs_port.c:26 | extern in lfs_port.h | LFS configuration struct |
| `g_w25q256_present` | bool | w25q256.c:11 | static | W25Q256 detection flag |
| `g_heap_free` | volatile uint32_t | app_sysmon.c:23 | static | Heap free bytes snapsho|
| `g_wm_display/carcontrol/imuservice/console` | volatile UBaseType_t | app_sysmon.c:24-27 | static | Stack watermarks |

### Global Constants

| Constant | Value | File | Description |
|----------|-------|------|-------------|
| `CPU_FREQ` | 480 | main.c:46 | CPU clock in MHz |
| `UART_FRAME_MAX_LEN` | 64 | uart_fifo.h:17 | Max UART3/4 frame byte length |
| `MAX_FRAME_SIZE` | 256 | uart_fifo.h:27 | Max legacy frame payload |
| `FRAME_HEADER` | 0xAA | uart_fifo.h:25 | Legacy frame start marker |
| `FRAME_TAIL` | 0x55 | uart_fifo.h:26 | Legacy frame end marker |
| `KEY_SCAN_INTERVAL_MS` | 5 | keyboard.h:35 | Keyboard scan period |
| `DEBOUNCE_TICKS` | 20 | keyboard.h:36 | Debounce timeout (ms) |

---

## 24. Function Call Graph / Dependency Map

```
main()
鈹溾攢 MPU_Config()
鈹溾攢 HAL_Init()
鈹溾攢 init_motor()                                     [Motor.c]
鈹? 鈹溾攢 Set_PID_Limit()                               [pid.c] 脳 4 motors
鈹? 鈹斺攢 Set_PID()                                     [pid.c] 脳 4 motors
鈹溾攢 SystemClock_Config()
鈹溾攢 MX_GPIO/TIM/USART/QSPI/SPI/UART/RAMECC_Init()
鈹溾攢 OLED_Init()                                      (if display enabled)
鈹溾攢 delay_init(480)                                  [delay.c]
鈹溾攢 IMU_init()                                       [IMU.c]
鈹? 鈹斺攢 setup_imu(1,1,1)                             [read_aux_data_mode.c]
鈹?    鈹溾攢 icm45686_read_regs / write_regs
鈹?    鈹溾攢 inv_imu_get_who_am_i
鈹?    鈹溾攢 inv_imu_soft_reset
鈹?    鈹溾攢 inv_imu_set_pin_config_int
鈹?    鈹溾攢 inv_imu_set_accel/gyro_fsr/frequency/ln_bw
鈹?    鈹斺攢 inv_imu_set_accel/gyro_mode
鈹溾攢 Start TIM2-5 encoders + TIM6
鈹溾攢 init_Car_Attitude()                              [car_attitude.c]
鈹? 鈹斺攢 Set_PID_Limit + Set_PID (pid_v_angle)
鈹溾攢 init_Car_Contorl()                               [car_control.c]
鈹? 鈹溾攢 Set_PID_Limit (pid_line_pos + pid_spin)
鈹? 鈹斺攢 Set_PID (pid_line_pos + pid_spin)
鈹溾攢 QSPI_W25Qxx_Init()                               [w25q256.c]
鈹? 鈹溾攢 W25Q256_Reset() (0x66+0x99)
鈹? 鈹斺攢 W25Q256_ReadID() 鈫?verify 0x4019
鈹溾攢 osKernelInitialize()
鈹溾攢 MX_FREERTOS_Init()                               [freertos.c]
鈹? 鈹溾攢 osSemaphoreNew 脳 2
鈹? 鈹溾攢 osMessageQueueNew
鈹? 鈹溾攢 xQueueCreate 脳 2 (UART3/4 frame)
鈹? 鈹溾攢 xStreamBufferCreate (console RX)
鈹? 鈹溾攢 osThreadNew 脳 12 (all CubeMX tasks)
鈹? 鈹溾攢 xTaskCreate(AppInit_Task)                     [app_init.c]
鈹? 鈹? 鈹溾攢 MX_USB_DEVICE_Init()
鈹? 鈹? 鈹溾攢 Set_Car_Control(1000,0,0)
鈹? 鈹? 鈹斺攢 vTaskDelete(NULL)
鈹? 鈹溾攢 Key_StartScanTask 鈫?xTaskCreate(vKeyScanTask) [keyboard.c]
鈹? 鈹斺攢 AppConsole_Init()                             [app_console.c]
鈹?    鈹斺攢 Console_RegisterCommands() 鈫?FreeRTOS_CLIRegisterCommand 脳 6
鈹斺攢 osKernelStart() 鈫?[never returns]
```

---

## 25. ISR-to-Task Data Flow

```
TIM6_DAC_IRQHandler                         (priority: medium)
  鈹?pwm_counter--
  鈹?compares: if pwm_counter < dutyX 鈫?GPIO SET/RESET
  鈹斺攢鈫?GPIO outputs: PWMA/PWMB/PWMC/PWMD pins (direct hardware control)

USB OTG_FS_IRQHandler 鈫?HAL_PCD_IRQHandler 鈫?CDC_Receive_FS callback
  鈹?feeds bytes into
  鈹斺攢鈫?xStreamBufferSendFromISR(console_rx_stream, ...)
       鈹斺攢鈫?Console_Task: xStreamBufferReceive() processes CLI commands

USART3_IRQHandler 鈫?HAL_UART_IRQHandler 鈫?HAL_UART_RxCpltCallback
  鈹?uart_fifo.c: line assembly on '\n'
  鈹斺攢鈫?xQueueSendFromISR(uart3_frame_queue, frame, ...)
       鈹斺攢鈫?Uart3Rx_Task: xQueueReceive() + frame processing

UART4_IRQHandler 鈫?HAL_UART_IRQHandler 鈫?HAL_UART_RxCpltCallback
  鈹?uart_fifo.c: line assembly on '\n'
  鈹斺攢鈫?xQueueSendFromISR(uart4_frame_queue, frame, ...)
       鈹斺攢鈫?Uart4Rx_Task: xQueueReceive() + frame processing

TIM2/3/4/5 encoder IRQs
  鈹?HAL_TIM_IRQHandler 鈫?encoder counter update (CubeMX-generated)
  鈹斺攢鈫?CarControl_Task reads encoder values every 10ms via Tim_Update_Enc_Speed()
```

---

## 26. Troubleshooting Guide

### Hardware Issues

| Symptom | Likely Cause | Debug Steps |
|---------|--------------|-------------|
| Motors don't spin | No PWM output | Check TIM6_DAC ISR code intact after CubeMX regeneration |
| Motors spin wrong direction | `motor.dir` or wiring | Swap `dir` flag for that motor in `init_motor()` |
| Motors spin at wrong speed | PID tuning | Check `motor_config.h` P/I/D values |
| IMU reads garbage | SPI4 not configured | Verify SPI4 pins + clock in CubeMX; check WHOAMI read |
| IMU yaw drifts | Gyro offset not calibrated | Wait 100+ samples stationary; check `calGyroVariance` threshold |
| OLED blank | SPI2 not initialized | Check `APP_DISPLAY_TYPE` setting in `app.h` |
| USB CDC not recognized | USB init timing | Ensure `MX_USB_DEVICE_Init()` runs AFTER `osKernelStart()` |
| Flash not detected | QSPI pins incorrect | Check JEDEC ID read: should return 0xEF4019 |
| Keyboard unresponsive | Scan task not created | Verify `Key_StartScanTask()` called in RTOS_THREADS |

### Software Issues

| Symptom | Likely Cause | Debug Steps |
|---------|--------------|-------------|
| `osDelay` timing 2脳 too long | Wrong `configCPU_CLOCK_HZ` | Must be `SystemCoreClock/2U` (240 MHz) for H750 |
| Stack overflow | Task stack too small | Check SysMon display for stack watermark values |
| Heap exhaustion | 80KB heap full | CLI: `sys` command shows heap free |
| `osKernelStart` hangs | USB init race condition | Set CubeMX USB init task to "none" |
| Encoder counts jump wildly | Noise on encoder lines | Check shielded wiring; enable digital filter on TIM inputs |
| LittleFS mount fails | Flash not formatted | CLI: `flash format` then `flash mount` |
| Car control overshoots | PID too aggressive | Reduce `P_POS` / `P_SPIN` in `control_config.h` |
| Car doesn't stop | `BIAS_LINE`/`BIAS_ANGLE` too large | Reduce bias values in `car_control.h` |

---

## 27. Boot Sequence Timeline

```
t=0ms:     Power-on reset 鈫?SystemInit() 鈫?main()
t=0ms:     MPU_Config(), HAL_Init()
t=1ms:     init_motor() 鈥?GPIO + PID config
t=2ms:     SystemClock_Config() 鈥?PLL lock to 480 MHz
t=3ms:     MX_GPIO_Init() through MX_RAMECC_Init()

t=100ms:   OLED_Init() (if enabled)
t=101ms:   HAL_Delay(1000) 鈥?1s warmup

t=1102ms:  delay_init(480) 鈥?DWT CYCCNT enabled
t=1103ms:  IMU_init() 鈥?ICM-45686 boot, WHOAMI check
t=1105ms:  HAL_Delay(1000) 鈥?1s IMU stabilization

t=2106ms:  Start TIM2-5 encoders + TIM6
t=2107ms:  init_Car_Attitude, init_Car_Contorl()
t=2108ms:  HAL_Delay(2000) 鈥?2s stabilization
t=2110ms:  printf("init ok") 鈥?via USB CDC

t=2112ms:  QSPI_W25Qxx_Init() 鈥?W25Q256 detect
t=2114ms:  HAL_Delay(1000) 鈥?1s

~t=3115ms: osKernelInitialize()
~t=3116ms: MX_FREERTOS_Init() 鈥?tasks, queues created
~t=3117ms: osKernelStart() 鈥?FreeRTOS scheduler begins

--- SCHEDULER RUNNING ---
~t=3118ms: AppInit_Task starts
~t=3120ms: MX_USB_DEVICE_Init() 鈥?USB enumeration begins
~t=3125ms: Set_Car_Control(1000, 0, 0) 鈥?first motion command
~t=3130ms: AppInit_Task self-deletes

~t=3120ms: LedBlink task starts (USB init duplicate, fine)
~t=3125ms: IMUService task starts (5ms sensor acquisition)
~t=3130ms: CarControl task starts (10ms control loop)
~t=3135ms: Console task starts (CLI prompt appears on USB CDC)
~t=3140ms: SysMon task starts (display shows heap + stack info)
```

---

## 28. Key Architectural Decisions

1. **Facade Pattern (`app.h`):** All CubeMX-generated files include only `app.h`, which combines all user module headers. This provides a single point of control for compile-time configuration (`APP_ENABLE_*` switches) and avoids circular dependencies.

2. **Software PWM via TIM6 ISR:** Instead of using hardware PWM timers, the system implements 4-channel software PWM in the TIM6 interrupt handler. This provides flexibility for pin assignments but creates a timing-critical ISR that must execute sub-microsecond GPIO comparisons.

3. **Incremental PID for Motor Speed:** The 4 motor speed loops use incremental (velocity form) PID. This accumulates output over time and avoids integral windup naturally, whereas positional PID requires explicit anti-windup limits.

4. **Positional PID for Motion Control:** The higher-level position/angle loops use positional PID with explicit anti-windup. This separates the cascaded control structure: position PID 鈫?velocity setpoint 鈫?velocity PID 鈫?PWM duty.

5. **Dynamic Encoder Selection (`Motor_Judge_Accuracy`):** In 4WD mode, the system dynamically selects the more accurate encoder for each side of the vehicle. This compensates for wheel slip by choosing the encoder whose current speed best matches the target.

6. **IMU-Aided Angular Velocity:** Car attitude uses the IMU gyro Z-axis directly for angular velocity, providing faster and more accurate rotation rate than encoder differential calculation.

7. **Static Task Allocation:** All 12 CubeMX tasks use static TCBs and stack buffers, reducing heap fragmentation and providing deterministic memory usage.

8. **FreeRTOS+CLI for Debug Console:** The USB CDC console uses FreeRTOS+CLI for command parsing, providing a clean interface for runtime system interrogation.

9. **LittleFS on QSPI Flash:** Uses the first 16 MB of the 32 MB W25Q256 for a wear-leveled filesystem, providing persistent configuration and log storage.

10. **Self-Deleting Init Task:** `AppInit_Task` runs once at startup (USB init + initial car command), then calls `vTaskDelete(NULL)` to free its 2KB stack, demonstrating resource-conscious design.

---

*Version: v3.1 | Updated: 2026-05-09*
*Documentation covers 45 source files across 6 architectural layers with line-by-line annotation.*
*Total: ~3100 lines of specification.*
