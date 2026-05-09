# Celebright 小车控制板开发模板

基于 STM32H750VBT6 + FreeRTOS 的小车运动控制开发模板，遵循"高内聚，低耦合"原则设计。

---

## 目录

1. [硬件平台](#1-硬件平台)
2. [软件架构](#2-软件架构)
3. [项目结构](#3-项目结构)
4. [模块剪裁指南](#4-模块剪裁指南)
5. [配置系统](#5-配置系统)
6. [FreeRTOS 任务配置](#6-freertos-任务配置)
7. [CubeMX 代码生成规范](#7-cubemx-代码生成规范)
8. [外设与 Pin Map](#8-外设与-pin-map)
9. [开发流程](#9-开发流程)
10. [重构记录](#10-重构记录)

---

## 1. 硬件平台

| 项目 | 规格 |
|------|------|
| MCU | STM32H750VBT6 (Cortex-M7, 480MHz, LQFP100) |
| IMU | ICM-45686 6 轴（SPI4） |
| Flash | W25Q256 32MB（QSPI BK1, 内存映射 0x90000000） |
| 电机 | 4 路直流电机 + 编码器 |
| 舵机 | SCS/FT 串行总线舵机（USART2, 1Mbps） |
| 显示 | OLED 128×64 / LCD TFT 240×280（SPI2） |
| 输入 | 4×4 矩阵键盘 |
| 通信 | USART1 printf / UART4 蓝牙 / USART3 帧协议 / USB OTG FS |
| 其他 | 蜂鸣器（TIM13）、2 路激光、4 路 LED |
| 调试 | SWD（PA13/PA14） |

---

## 2. 软件架构

### 2.1 分层架构

```
┌─────────────────────────────────────────────────────────────────┐
│                    应用层（Application）                          │
│                                                                 │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌───────────────┐      │
│  │  显示     │ │ 键盘输入  │ │ 音乐播放  │ │  UART 通信     │      │
│  │ OLED/LCD │ │ 4×4 矩阵  │ │ 蜂鸣器    │ │ 帧协议/蓝牙    │      │
│  └──────────┘ └──────────┘ └──────────┘ └───────────────┘      │
│                                                                 │
│  ┌──────────────────────────┐ ┌──────────────────────────────┐ │
│  │  机械臂子系统             │ │  2D 激光云台                  │ │
│  │  ARobot → arm 运动学     │ │  holder2D → SCS 舵机          │ │
│  │        → roboticArm 舵机 │ │                               │ │
│  └──────────────────────────┘ └──────────────────────────────┘ │
├─────────────────────────────────────────────────────────────────┤
│                   控制层（Control, 10ms）                         │
│                                                                 │
│  car_control ──位置 PID──→ Set_Car_Attitude(v_line, v_angle)    │
│       │                           │                              │
│       │  car_state_t               ▼                              │
│       │  (yaw/yaw_all/yaw_circles) car_attitude ──角速度 PID     │
│       │                           │                              │
│       └──────── 共享状态 ────────→│                              │
│                                   ▼                              │
│                        Wheel_Set_V_Real(v_left, v_right)          │
│                                   │                              │
│                    Motor×4 ──增量 PID──→ PWM ──→ GPIO            │
│                    (编码器反馈闭环)                               │
├─────────────────────────────────────────────────────────────────┤
│                  传感器层（Sensor, 5ms）                          │
│                                                                 │
│  ┌─────────────────────────┐  ┌──────────────────────────────┐ │
│  │ IMU (ICM-45686, SPI4)   │  │ 编码器 (TIM2~5)               │ │
│  │ Madgwick AHRS           │  │ 4 轮速度测量                  │ │
│  │ → yaw/pitch/roll        │  │ → motor.EncSource             │ │
│  │ → imu_g_z (角速度)      │  │                               │ │
│  └─────────────────────────┘  └──────────────────────────────┘ │
├─────────────────────────────────────────────────────────────────┤
│                 硬件抽象层（HAL + CMSIS）                         │
│  GPIO │ UART×5 │ SPI×2 │ QSPI │ TIM×6 │ USB OTG                │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 Facade 模式：`app.h`

所有 CubeMX 生成的 `.c` 文件（`main.c`、`freertos.c`）**仅** `#include "app.h"` 一行，不再逐个 include 各模块头文件。

```
main.c ──→ #include "app.h"
freertos.c ──→ #include "app.h"

app.h 内部组织（按层）:
  ├── Common:   config.h → {platform_config, motor_config, control_config, rtos_config}
  │             delay.h, sys_time.h
  ├── Control:  pid.h, PWM.h, Motor.h, car_attitude.h, car_control.h
  ├── Sensor:   inv_imu_*.h, read_aux_data_mode.h, IMU.h
  ├── Device:   uart_fifo.h, oled_spi_V0.2.h, lcd_model.h, keyboard.h, MusicPlayer.h
  ├── Robot:    SCServo.h, arm.h, roboticArm.h, ARobot.h, holder2D.h
  └── Shared:   car_state_t, extern 句柄（队列/信号量/IMU数据/PWM占空比）
```

**重要规则**：

| 文件类型 | Include 方式 |
|----------|-------------|
| CubeMX `.c`（main.c, freertos.c） | **仅** `#include "app.h"` |
| ISR 文件（stm32h7xx_it.c） | 按需 include（如 `#include "PWM.h"`） |
| 用户模块 `.c` | `#include "app.h"` 或按需 include 个别模块 |
| 纯数据头文件（oledfont.h, MusicScore.h 等） | **不进入 app.h**，由使用者的 `.c` 自行 include |

**Thin wrapper 头文件** — 每个域一个，方便 IDE 导航，内部仅一行 `#include "app.h"`：

| Wrapper | 覆盖域 |
|---------|--------|
| `app_common.h` | Common |
| `app_control.h` | Control |
| `app_sensor.h` | Sensor |
| `app_device.h` | Device |
| `app_robot.h` | Robot |

### 2.3 控制数据流

```
用户命令 Set_Car_Control(x, y, angle)
    │
    ▼
car_control (位置 PID)            ← 编码器里程计反馈
    │  输出: 目标 v_line, v_angle
    ▼
car_attitude (速度控制)           ← IMU z 轴角速度反馈
    │  输出: v_left, v_right (mm/s)
    ▼
Motor ×4 (增量 PID 速度环)        ← TIM2~5 编码器反馈
    │  输出: 占空比 duty ×4
    ▼
PWM 软件生成 (TIM6 ISR 50Hz)     → GPIO PWMA/B/C/D
    │
    ▼
直流电机 + 编码器 ×4
```

### 2.4 软件 PWM 原理

TIM6 以 40kHz（25μs）触发中断。ISR 中计数 500→0 循环（20ms = 50Hz）。每个通道在计数值 < 目标占空比时 GPIO 置高。4 通道独立，分辨率 1/500。

> 软件 PWM 消耗 CPU（40kHz ISR），但节省硬件定时器资源。

---

## 3. 项目结构

### 3.1 目录树

```
STM32H750VBT6_Celebright_Template_V3/
├── STMH750VBT6_Celebright_V3.ioc      # [CubeMX] 引脚/外设配置
│
├── Core/
│   ├── Inc/                            # 头文件
│   │   ├── main.h                      # [CubeMX] Pin 宏 + 错误声明
│   │   ├── FreeRTOSConfig.h            # [CubeMX] FreeRTOS 内核参数
│   │   ├── stm32h7xx_hal_conf.h        # [CubeMX] HAL 模块使能
│   │   ├── stm32h7xx_it.h              # [CubeMX] ISR 声明
│   │   ├── gpio.h / tim.h / usart.h    # [CubeMX] 外设初始化声明
│   │   ├── spi.h / quadspi.h / ...     # [CubeMX] 外设初始化声明
│   │   │
│   │   ├── app.h                       # ★ Facade 统一入口
│   │   ├── app_control.h / app_sensor.h / app_device.h
│   │   ├── app_robot.h / app_common.h  # ★ Thin wrapper
│   │   │
│   │   ├── config.h                    # 配置聚合器（向后兼容）
│   │   ├── platform_config.h           # 平台参数（轮径/轮距/编码器）
│   │   ├── motor_config.h              # 电机 PID 参数
│   │   ├── control_config.h            # 控制 PID 参数
│   │   ├── rtos_config.h               # 任务周期 / 模式开关
│   │   │
│   │   ├── pid.h / PWM.h / Motor.h     # 控制域
│   │   ├── car_attitude.h / car_control.h
│   │   ├── IMU.h / inv_imu_*.h         # 传感器域
│   │   ├── read_aux_data_mode.h
│   │   ├── oled_spi_V0.2.h / lcd_model.h # 设备域
│   │   ├── lcd_st7789.h / display_service.h  # 显示抽象层
│   │   ├── keyboard.h / MusicPlayer.h
│   │   ├── uart_fifo.h / delay.h / sys_time.h
│   │   ├── app_init.h / app_control.h / app_sensor.h    # 应用服务（按域）
│   │   ├── app_device.h / app_console.h / app_sysmon.h
│   │   ├── app_rtos_hooks.h / app_qspi.h
│   │   ├── FreeRTOS_CLI.h
│   │   ├── w25q256.h / lfs.h / lfs_util.h / lfs_port.h  # Flash + 文件系统
│   │   ├── arm.h / ARobot.h / roboticArm.h / holder2D.h  # 机器臂域
│   │   ├── oledfont.h / oledpicture_V0.2.h  # 数据（不通过 app.h）
│   │   ├── lcd_fonts.h / lcd_image.h
│   │   └── MusicScore.h
│   │
│   └── Src/                            # 源文件
│       ├── main.c / freertos.c         # [CubeMX] + USER CODE
│       ├── stm32h7xx_it.c / hal_msp.c  # [CubeMX] + USER CODE
│       ├── system_stm32h7xx.c          # [CubeMX]
│       ├── gpio.c / tim.c / usart.c / spi.c / quadspi.c / ...
│       │
│       ├── pid.c / PWM.c / Motor.c     # 控制域
│       ├── car_attitude.c / car_control.c
│       ├── IMU.c / inv_imu_driver.c / inv_imu_transport.c
│       ├── read_aux_data_mode.c
│       ├── oled_spi_V0.2.c / lcd_model.c / lcd_fonts.c / lcd_image.c
│       ├── lcd_st7789.c / display_service.c          # 显示抽象层
│       ├── keyboard.c / MusicPlayer.c / uart_fifo.c
│       ├── delay.c / sys_time.c
│       ├── app_init.c / app_control.c / app_sensor.c  # 应用服务
│       ├── app_device.c / app_console.c / app_sysmon.c
│       ├── app_rtos_hooks.c / app_qspi.c
│       ├── FreeRTOS_CLI.c
│       ├── w25q256.c / lfs.c / lfs_util.c / lfs_port.c  # Flash + 文件系统
│       ├── arm.c / ARobot.c / roboticArm.c / holder2D.c
│       └── ...
│
├── Drivers/  (CMSIS + STM32H7xx_HAL)
├── Middlewares/  (FreeRTOS v10.3.1)
├── SCSLib/  (第三方 SCS 舵机协议库)
└── MDK-ARM/  (Keil 项目文件)
```

> `[CubeMX]` = 生成文件，修改仅限 `USER CODE BEGIN/END` 范围
> `★` = Phase 2-3 重构新增

---

## 4. 模块剪裁指南

项目按功能域组织，每个域可独立启用/裁剪。

### 4.1 剪裁操作矩阵

下表列出每个模块的开关位置和依赖关系。

| 模块 | 开关方式 | 依赖 | 裁剪步骤 |
|------|----------|------|----------|
| **小车运动控制** (car_control + car_attitude + Motor) | `rtos_config.h`: `USE_CAR_CONTROL = 0` 关闭位置 PID<br>`rtos_config.h`: `V_ANGLE_PID = 0` 关闭角速度 PID | 无（核心） | 修改宏即可，不影响编译 |
| **IMU 传感器** | CubeMX 禁用 SPI4 → 移除 `inv_imu_*`, `read_aux_data_mode`, `IMU` 的 `.c` 文件 | car_attitude 角速度可选用编码器差速 | 1. CubeMX 关闭 SPI4<br>2. 从 Keil 项目移除 4 个 `.c`<br>3. `app.h` 注释 Sensor 层 include |
| **OLED 显示** | 1. `app.h` 注释 `#include "oled_spi_V0.2.h"`<br>2. `freertos.c` 注释 `Task_Screen` 任务创建 | 无 | 仅注释，编译通过 |
| **LCD 显示** | 1. `app.h` 注释 `#include "lcd_model.h"`<br>2. 从 Keil 移除 `lcd_model.c`, `lcd_fonts.c`, `lcd_image.c` | 无 | 注释 + 移除 .c |
| **矩阵键盘** | 1. `app.h` 注释 `#include "keyboard.h"`<br>2. `freertos.c` RTOS_THREADS 注释 `Key_StartScanTask()` | 无 | 注释即可 |
| **蜂鸣器音乐** | 1. `app.h` 注释 `#include "MusicPlayer.h"`<br>2. `freertos.c` 注释 `Task_Buzzer` 任务创建 | 无 | 注释即可 |
| **UART 帧通信** | 1. `app.h` 注释 `#include "uart_fifo.h"`<br>2. `freertos.c` 注释 `Uart3Handle/Uart4Handle` 任务 + 队列创建 | 无 | 注释即可 |
| **蓝牙广播** | `freertos.c` 注释 `Task_Broadcast` 任务创建 | 无 | 注释即可 |
| **机械臂子系统** (arm + ARobot + roboticArm) | 1. `app.h` 注释 Robot 层 include<br>2. 从 Keil 移除 `arm.c`, `ARobot.c`, `roboticArm.c`<br>3. `main.c` 注释机械臂初始化代码 | SCSLib（如果也剪裁舵机） | 注释 + 移除 .c |
| **2D 激光云台** (holder2D) | 1. `app.h` 注释 `#include "holder2D.h"`<br>2. 从 Keil 移除 `holder2D.c` | SCSLib | 注释 + 移除 .c |
| **SCS 舵机协议库** | CubeMX 禁用 USART2 → 从 Keil 移除 `SCSLib` 组所有 `.c`，从 include path 移除 `../SCSLib` | 被机械臂/云台依赖 | 仅在剪裁机械臂+云台后可剪裁 |
| **W25Q256 Flash** | `main.c` 注释 `QSPI_W25Qxx_Init()` 调用 | 无 | 注释即可 |
| **USB OTG** | CubeMX 禁用 USB_OTG_FS | 无 | CubeMX 操作为主 |

### 4.2 典型裁剪场景

#### 场景 A：纯底盘（电机+编码器+IMU，不要显示/通信/机械臂）

```
操作清单:
  1. app.h: 注释 Device 层、Robot 层所有 #include
  2. freertos.c: 注释 Task_Screen, Task_Key, Task_Buzzer,
                 Task_Broadcast, Task_Uart3Handle, Task_Uart4Handle,
                 对应的队列创建
  3. Keil 项目移除: oled_spi_V0.2.c, lcd_*.c, keyboard.c, MusicPlayer.c,
                   uart_fifo.c, arm.c, ARobot.c, roboticArm.c, holder2D.c
  4. main.c: 注释 OLED_Init(), MusicPlayer_Init(), 机械臂初始化
```

#### 场景 B：走直线测试（只保留控制闭环 + IMU）

```
  只需: rtos_config.h 保持 USE_CAR_CONTROL=1
  Task_APP 中调用 Set_Car_Control(1000, 0, 0) 直行 1 米
```

#### 场景 C：全功能（当前默认）

```
  不做任何修改，所有模块启用
```

---

## 5. 配置系统

### 5.1 配置文件分层

```
config.h  ←── 聚合器（向后兼容，include 下面 4 个）

├── platform_config.h    # 小车几何参数 + 速度上限
├── motor_config.h       # 4 电机 PID + 限幅
├── control_config.h     # 位置/角度/角速度 PID + 限幅
└── rtos_config.h        # 任务周期 + 模式开关
```

推荐新代码按域 include：`#include "platform_config.h"` 等。旧代码通过 `config.h` 向后兼容。

### 5.2 platform_config.h — 平台几何参数

```c
/* 轮子 */
#define WHEEL_DIR         47F             // 轮径 mm
#define WHEEL_PERIMETER   148.722996F     // 轮周长 mm

/* 车体 */
#define FRAME_W_HALF      66.25F          // 左右轮距一半 mm
#define FRAME_L_HALF      55.85F          // 前后轴距一半 mm

/* 编码器 */
#define ENC_EVERY_CIRCLE  1061.268        // 每圈脉冲数
#define USE_4_TIMES_ENCODER 1             // 四倍频

/* 速度上限 */
#define MAX_V_ENC         (2 * ENC_EVERY_CIRCLE)  // 编码器速度上限
#define MAX_V_REAL        (MAX_V_ENC / (ENC_EVERY_CIRCLE / WHEEL_PERIMETER))  // mm/s
#define MAX_V_ANGLE       60.0F           // 最大角速度 °/s
```

### 5.3 motor_config.h — 电机 PID

```c
#define MAX_MOTOR_DUTY    0.999F          // PWM 最大占空比

#define P_LF 0.02F  I_LF 0.01F  D_LF 0.015F   // 左前
#define P_LR 0.02F  I_LR 0.01F  D_LR 0.015F   // 左后
#define P_RF 0.02F  I_RF 0.01F  D_RF 0.015F   // 右前
#define P_RR 0.02F  I_RR 0.01F  D_RR 0.015F   // 右后
```

### 5.4 control_config.h — 控制 PID

```c
#define P_POS  2.0F   I_POS  0.01F   D_POS  0.1F    // 位置环
#define P_SPIN 2.0F   I_SPIN 0.01F   D_SPIN 0.5F    // 旋转环
#define P_V_ANGLE 1.0F  I_V_ANGLE 0.63F  D_V_ANGLE 0.01F  // 角速度环
```

### 5.5 rtos_config.h — 任务周期与模式开关

```c
/* 任务周期 (ms) */
#define TASK_ITV_CAR    10              // 控制任务周期
#define TASK_ITV_IMU    5               // IMU 任务周期
#define TASK_ITV_UPLOAD 50              // 上传周期

/* 模式开关 */
#define USE_CAR_CONTROL 1               // 1=位置PID闭环, 0=仅姿态环
#define V_ANGLE_PID      1               // 1=角速度PID, 0=不控角速度
#define V_DEGREE_FROM_IMU 1             // 1=IMU角速度, 0=编码器差速
```

### 5.6 app.h — 构建时配置

```c
#define APP_DISPLAY_TYPE  DISPLAY_TYPE_OLED   // OLED / LCD
```

---

## 6. FreeRTOS 任务配置

FreeRTOS v10.3.1，CMSIS_V2 API，静态内存分配，1000Hz Tick。

### 6.1 任务列表

| 任务名 | 优先级 | 栈(字) | 周期 | 所属域 | 功能 |
|--------|--------|--------|------|--------|------|
| `IMUService` | Normal3(27) | 128 | 5ms | Sensor | 读取 IMU 姿态 + 陀螺仪数据 |
| `CarControl` | Normal5(29) | 128 | 10ms | Control | 编码器更新 → 电机 PID → 控制 PID → 姿态输出 |
| `KeyScan` | Normal6(30) | 128 | idle | Device | 矩阵键盘扫描 |
| `LEDBlink` | AboveNormal(32) | 128 | 1s | — | LED0 闪烁 |
| `Uart3Rx` | Normal7(31) | 256 | block | Device | UART3 帧解析 |
| `Uart4Rx` | Normal7(31) | 256 | block | Device | UART4 帧解析 |
| `Broadcast` | Normal1(25) | 256 | 50ms | Device | 蓝牙广播 |
| `Display` | Normal1(25) | 128 | 100ms | Device | OLED/LCD 显示 |
| `Buzzer` | Normal1(25) | 128 | idle | Device | 蜂鸣器（暂未启用） |
| `Reserved` | Normal1(25) | 1024 | 50ms | — | 待开发占位 |
| `SysMon` | Normal1(25) | 256 | 5s | System | 系统监视器（堆/栈水位，OLED 显示） |
| `Console` | Normal1(25) | 512 | block | System | USB CDC CLI 控制台（FreeRTOS+CLI） |
| `Task_APP` | Normal5(29) | 512 | once | — | 一次性初始化后自删除 |

### 6.2 RTOS 对象

| 对象 | 类型 | 参数 | 用途 |
|------|------|------|------|
| `queue_key` | CMSIS MessageQ | 1 × char | 按键传递 |
| `uart3_frame_queue` | FreeRTOS Queue | 5 × 64B | UART3 帧缓冲 |
| `uart4_frame_queue` | FreeRTOS Queue | 5 × 64B | UART4 帧缓冲 |
| `semphr_buzzer_trigger` | Binary semaphore | initial=1 | 蜂鸣器触发 |
| `semphr_uart_receive` | Counting semaphore | max=16 | UART 接收通知 |

### 6.3 内核参数

| 参数 | 值 |
|------|-----|
| Heap Size | 81920 bytes |
| Max Priorities | 56 |
| FPU | 开启 |
| Mutex / RecursiveMutex / CountingSem | 使能 |

### 6.4 中断优先级规则

FreeRTOS 的 SysTick/PendSV 优先级必须最低（15）。所有硬件中断优先级必须更高（数值 < 15），确保 ISR 中可安全调用 FreeRTOS API（`xQueueSendFromISR` 等）。

| IRQ | 优先级 |
|-----|--------|
| TIM6_DAC | 5 |
| UART4 | 5 |
| USART3 | 5 |
| SysTick | 15 |
| PendSV | 15 |

---

## 7. CubeMX 代码生成规范

### 7.1 核心规则

所有 CubeMX 生成的文件通过 `USER CODE BEGIN/END` 标记保护用户代码。重新生成时：

- 标记内的代码**保留**
- 标记外的代码**覆盖**

### 7.2 可修改区域

| 文件 | 可修改区域 |
|------|-----------|
| `main.c` | `Includes` / `0` / `1` / `2` / `3` / `4` / `Init` / `SysInit` / `WHILE` / `PV` |
| `freertos.c` | `Includes` / `Variables` / `RTOS_QUEUES` / `RTOS_THREADS` / `Application` / 各 `Task_xxx` 函数体 |
| `stm32h7xx_it.c` | `Includes` / 各 `xxx_IRQn 0` / `1` |
| `stm32h7xx_hal_msp.c` | `xxx_MspInit 0/1` / `xxx_MspDeInit 0/1` |
| `gpio.c` / `tim.c` / `usart.c` / `spi.c` | `xxx_Init 1/2` |

### 7.3 CubeMX 再生后恢复清单

1. MDK include path 恢复 `../SCSLib`
2. MDK 项目添加 `sys_time.c`
3. 确认 `read_aux_data_mode.c` 在项目中

### 7.4 USB 初始化说明

CubeMX 会在 `LedBlink_Handler` 中自动插入 `MX_USB_DEVICE_Init()`。
为避免阻塞，在 CubeMX → USB_DEVICE → Platform Settings 中将 task 设为 "none"。
USB 初始化由 `AppInit_Task`（`app_init.c`）统一处理。

---

## 8. 外设与 Pin Map

### 8.1 定时器

| 定时器 | 模式 | 引脚 | 用途 |
|--------|------|------|------|
| TIM2 | 编码器 32-bit | PA15-CH1, PB3-CH2 | 左后轮 |
| TIM3 | 编码器 16-bit | PC6-CH1, PC7-CH2 | 左前轮 |
| TIM4 | 编码器 16-bit | PD12-CH1, PB7-CH2 | 右前轮 |
| TIM5 | 编码器 32-bit | PA0-CH1, PA1-CH2 | 右后轮 |
| TIM6 | 基本定时器 40kHz | 内部 | 软件 PWM 时基 |
| TIM13 | PWM CH1 | PA6 | 蜂鸣器 |

### 8.2 串口

| 串口 | TX | RX | 波特率 | 用途 |
|------|----|----|--------|------|
| USART1 | PB14 | PB15 | 115200 | printf 调试 |
| USART2 | PD5 | PA3 | 1M | SCS 舵机总线 |
| USART3 | PD8 | PB11 | 115200 | 帧协议接收 |
| UART4 | PB9 | PD0 | 115200 | 蓝牙广播+接收 |
| UART5 | PB13 | PD2 | 9600 | 预留 |

### 8.3 SPI / QSPI

| 外设 | 引脚 | 速率 | 用途 |
|------|------|------|------|
| SPI2 | PA9-SCK, PC1-MOSI, PB12-NSS | 25Mbps | OLED + LCD |
| SPI4 | PE12-SCK, PE6-MOSI, PE5-MISO | 15Mbps | ICM-45686 IMU |
| QSPI | PB2-CLK, PB6-NCS, PD11-IO0, PC10-IO1, PE2-IO2, PD13-IO3 | — | W25Q256 Flash |

### 8.4 Pin Map 速查

**电机方向：** A(PE3/PC13), B(PC0/PC3), C(PC4/PC5), D(PB1/PE7)

**PWM 输出：** A(PB0), B(PB10), C(PA2), D(PD14)

**IMU：** CS(PE4)

**键盘：** ROW(PE9/PE11/PE13/PE14), COL(PA10/PB5/PC9/PC8)

**其他：** LED0(PD3), LED1(PD4), LED2(PE0), LED3(PE1), User(PD15), Laser(PA8), Laser2(PC12), Buzzer(PA6)

---

## 9. 开发流程

### 9.1 新增用户模块

1. 在 `Core/Inc/` 创建 `xxx.h`，`Core/Src/` 创建 `xxx.c`
2. 在 `app.h` 对应域中添加 `#include "xxx.h"`
3. 如需暴露跨模块数据，在 `app.h` 底部添加 `extern` 声明
4. 将 `.c` 文件添加到 Keil 项目（`Application/User/bsp` 组）

### 9.2 PID 参数整定

1. 在 `motor_config.h` 调整各电机 P/I/D 值
2. 在 `control_config.h` 调整位置/角度/角速度环 P/I/D 值
3. 速度范围参考：最大编码器速度 ≈ 2122 enc/s ≈ 299 mm/s

### 9.3 初始化流程

```
HAL_Init() → init_motor()
  → SystemClock_Config()
  → MX_GPIO/TIM/USART/SPI/QUADSPI_Init()
  → USER CODE 2:
      OLED_Init() → delay_init(480) → IMU_init()
      → HAL_TIM_Encoder_Start_IT(TIM2~5)
      → HAL_TIM_Base_Start_IT(TIM6)
      → init_Car_Attitude() → init_Car_Contorl()
      → QSPI_W25Qxx_Init()
  → osKernelInitialize() → MX_FREERTOS_Init() → osKernelStart()
```

### 9.4 初始化后执行

- `Task_APP`（一次性）: 调用 `Set_Car_Control(1000,0,0)` 直行 1m → 自删除
- `IMUService` (5ms): 持续读取 IMU，更新姿态
- `CarControl` (10ms): 编码器 → 电机 PID → 控制 PID → 姿态输出

---

## 10. 重构记录

### Phase 1 (2026-05-04) — Bug 修复与结构矫正
- `Motor.h` 移至 `Core/Inc/`，`read_aux_data_mode.h` 新建
- `holder2D.h` / `roboticArm.h` 头文件保护修复
- `angle_limits[]` static→extern 修正
- SCSLib 路径统一 + MDK include path 添加

### Phase 2 (2026-05-04) — Facade 模式
- 新建 `app.h` + 5 个 thin wrapper
- `main.c` / `freertos.c` includes 从多行简化为一行
- `car_state_t` 共享结构体声明

### Phase 3 (2026-05-04) — 解耦
- 消除 car_attitude ↔ car_control 循环依赖（通过 car_state）
- 拆分 `config.h` → 4 个域配置文件
- `sys_time.h/c` 新建，`nowtime` 从 main.c 迁移

### Phase 3 Errata (2026-05-04) — 编译修复
- `arm.h` L1/L2/L3 → link1/link2/link3（避让 MusicPlayer 音符宏）
- 数据头文件（oledfont/MusicScore等）从 app.h 移除
- sys_time.c 加入 Keil 项目

### Phase 4 (2026-05-05) — Thin Delegation 模式

参照 freeRTOS Template，将所有 freertos.c Handler 的业务逻辑抽取到独立的 `app_*.c` 服务文件中。

| 变更 | 说明 |
|------|------|
| CubeMX 任务重命名 | `_9axisService`→`IMUService`, `CarAttitude`→`CarControl`, `Key`→`KeyScan`, `Screen_LCD`→`Screen`, `todo`→`Reserved`, `Uart4Handle`→`Uart4Rx`, `Uart3Handle`→`Uart3Rx` |
| Handler 薄委托 | 10 个 Handler body 均改为单行 `AppXxx_Task(argument)` |
| 新建 `app_init.c` | `AppInit_Task()` — 原 `Task_APP`，一次性初始化后自删除 |
| 新建 `app_control.c` | `AppCarControl_Task()` — 10ms 控制闭环（编码器→PID→PWM） |
| 新建 `app_sensor.c` | `AppIMUService_Task()` — 5ms IMU 数据采集+姿态更新 |
| 新建 `app_device.c` | 6 个设备任务：`AppScreen/Broadcast/Uart3Rx/Uart4Rx/KeyScan/Buzzer/Reserved_Task` |
| `app.h` 更新 | 添加 9 个 `AppXxx_Task` 函数声明 |
| `freertos.c` Application 清理 | 移除原内联 `Task_APP` 函数体（已迁移到 `app_init.c`） |

**Handler 对比（CarControl 为例）**：
```c
// 改造前：freertos.c 内 15 行业务逻辑
void Task_CarAttitude(void *argument) {
    TickType_t xLastWakeTime = ...;
    for(;;) { vTaskDelayUntil(...); Tim_Update_Enc_Speed(); ... }
}

// 改造后：一行委托
void CarControl_Handler(void *argument) {
    AppCarControl_Task(argument);
}
```

**收益**：
- `freertos.c` 从 498 行缩减至 447 行，所有 USER CODE 均为纯委托调用
- CubeMX 再生后只需在 Handler 内填写一行 `AppXxx_Task(argument)` 即可恢复
- 业务逻辑集中于 `app_*.c`，按域拆分，职责清晰

### Phase 5 (v3.1) — 大规模重构

- **Facade 模式**：`app.h` 统一 include
- **Thin delegation**：所有 `freertos.c` handler 为单行 `AppXxx_Task(argument)`
- **LCD 驱动提取**：从 `spi.c` 中独立为 `lcd_st7789.c/h`
- **W25Q256 + LittleFS**：从参考模板移植（`w25q256.c/h`、`lfs.c/h`、`lfs_util.c/h`、`lfs_port.c/h`）
- **USB CDC CLI 控制台**：`app_console.c/h`（FreeRTOS+CLI）
- **系统监视器**：`app_sysmon.c/h`（堆/栈水位，OLED 显示）
- **FreeRTOS 运行时统计**：DWT→CYCCNT，无需额外定时器
- **显示抽象层**：`display_service.c/h`，编译时 OLED/LCD 切换
- **模块裁剪**：`APP_ENABLE_*` 宏开关
- **全代码英文注释**
- **Bug 修复**：L1/L2/L3 命名冲突、`angle_limits` static→extern、`holder2D` 头文件保护、**MPU NO_ACCESS→FULL_ACCESS（修复 osKernelStart 后 isb 死锁）**

---

## CubeMX 配置与代码一致性审计 (v3.1)

### 需在 CubeMX 中修改

| 对象 | 当前 CubeMX | 应改为 | 原因 |
|------|-----------|--------|------|
| Console 任务优先级 | `osPriorityLow` | `osPriorityNormal1` | 低优先导致 CLI 响应极慢 |
| USB_DEVICE 初始化任务 | LEDBlink | **"none"** | 避免 `MX_USB_DEVICE_Init()` 阻塞 LED 任务 |

### CubeMX 不支持的对象（代码中手动创建）

| 对象 | 创建方式 | 原因 |
|------|---------|------|
| `uart3_frame_queue` | `xQueueCreate(5,64)` in RTOS_QUEUES | CubeMX CMSIS Queue 可选替代，需同步改 uart_fifo API |
| `uart4_frame_queue` | `xQueueCreate(5,64)` in RTOS_QUEUES | 同上 |
| `console_rx_stream` | `xStreamBufferCreate(256,1)` in RTOS_QUEUES | CubeMX 不支持 StreamBuffer |
| `AppInit_Task` | `xTaskCreate(AppInit,512)` in RTOS_THREADS | 自删除一次性任务 |

### CubeMX 中已正确配置的对象

| 对象 | 类型 | 参数 |
|------|------|------|
| `queue_key` | CMSIS MessageQueue | 1 × char |
| `semphr_buzzer_trigger` | Binary Semaphore | initial=1, max=1 |
| `semphr_uart_receive` | Counting Semaphore | max=16, initial=0 |
| 12 个 FreeRTOS Tasks | Static Allocation | 各配 stack/priority |

### 可选添加（如需）

| 对象 | 类型 | 用途 |
|------|------|------|
| `UsbTxMutex` | CMSIS Mutex | 多任务共用 USB CDC TX 时互斥保护（当前仅 Console 使用 USB，暂无冲突） |
| `LCDMutex` | CMSIS Mutex | 多任务共用 LCD 时互斥保护（当前仅 SysMon 写屏，暂无冲突） |

---

## 许可

Copyright (c) 2023-2025 Celebright Team

---

*最后更新：2026-05-09*
