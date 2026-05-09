# Celebright 小车控制板 — 系统规格文档

**MCU**: STM32H750VBT6 (Cortex-M7, 480MHz)  
**RTOS**: FreeRTOS v10.3.1 (CMSIS_V2, Static Allocation)  
**Toolchain**: MDK-ARM V5.32 / CubeMX 6.14.1 / ARM Compiler 5  

---

## 1. 项目架构

### 1.1 分层架构

```
┌──────────────────────────────────────────────────────────────┐
│ Application                                                │
│ app_init.c          ── One-shot startup                    │
│ app_control.c       ── 10ms control loop                   │
│ app_sensor.c        ── 5ms IMU service                     │
│ app_device.c        ── Display / Broadcast / UART / Key    │
│ app_sysmon.c        ── System monitor (heap/stack)         │
│ app_console.c       ── USB CDC CLI console                 │
│ app_qspi.c          ── W25Q256 + LittleFS                  │
│ app_rtos_hooks.c    ── FreeRTOS hooks + runtime stats      │
├──────────────────────────────────────────────────────────────┤
│ Control (always enabled)                                   │
│ pid / PWM / Motor / car_attitude / car_control            │
├──────────────────────────────────────────────────────────────┤
│ Sensor                                                     │
│ IMU (ICM-45686) / inv_imu_driver / read_aux_data_mode    │
├──────────────────────────────────────────────────────────────┤
│ Device                                                     │
│ OLED/LCD / keyboard / MusicPlayer / uart_fifo               │
├──────────────────────────────────────────────────────────────┤
│ Robot                                                      │
│ arm / ARobot / roboticArm / holder2D / SCSLib               │
├──────────────────────────────────────────────────────────────┤
│ Storage                                                    │
│ W25Q256 + LittleFS + FreeRTOS+CLI                           │
├──────────────────────────────────────────────────────────────┤
│ HAL + CMSIS                                                │
│ GPIO / UART / SPI / QSPI / TIM / USB OTG                   │
└──────────────────────────────────────────────────────────────┘
```

### 1.2 Facade 模式

`app.h` 是项目的**唯一统一入口**。所有 CubeMX 生成文件仅 `#include "app.h"`。

```
main.c ──→ #include "app.h"
freertos.c ──→ #include "app.h"

app.h includes (in order):
  Common:    config.h → {platform,motor,control,rtos}_config.h, delay.h, sys_time.h
  Control:   pid.h, PWM.h, Motor.h, car_attitude.h, car_control.h
  Sensor:    inv_imu_*.h, read_aux_data_mode.h, IMU.h           [if APP_ENABLE_IMU]
  Device:    uart_fifo.h, oled_spi_V0.2.h, lcd_st7789.h, ...   [if APP_ENABLE_*]
  Robot:     SCServo.h, arm.h, roboticArm.h, ...               [if APP_ENABLE_ROBOTIC_ARM]
```

### 1.3 Thin Delegation 模式

`freertos.c` 的每个 Handler 仅 **一行委托调用**：

```c
void CarControl_Handler(void *argument) {
    AppCarControl_Task(argument);   // ← 唯一一行
}
```

业务逻辑在 `app_control.c` 的 `AppCarControl_Task()` 中。CubeMX 再生时只需恢复这一行。

---

## 2. 目录结构

```
STM32H750VBT6_Celebright_Template_V3/
│
├── STMH750VBT6_Celebright_V3.ioc          [CubeMX] 引脚/外设/RTOS 总配置
├── .mxproject                              [CubeMX] 项目元数据
├── README.md                               [Doc] 项目总览
├── Specification.md                        [Doc] 本文档
│
├── Core/
│   ├── Inc/                                # 头文件
│   │   ├── app.h                           ★ Facade 统一入口
│   │   ├── app_control.h                    Thin wrapper (仅 include app.h)
│   │   ├── app_sensor.h                     Thin wrapper
│   │   ├── app_device.h                     Thin wrapper
│   │   ├── app_robot.h                      Thin wrapper
│   │   ├── app_common.h                     Thin wrapper
│   │   ├── app_init.h                       Thin wrapper
│   │   ├── app_console.h                    Thin wrapper
│   │   ├── app_sysmon.h                     Thin wrapper
│   │   ├── app_qspi.h                       Thin wrapper
│   │   ├── app_rtos_hooks.h                Thin wrapper
│   │   ├── display_service.h               ★ 显示抽象层
│   │   ├── config.h                         配置聚合器（include 4 个子文件）
│   │   ├── platform_config.h                ★ 小车几何参数 + 速度上限
│   │   ├── motor_config.h                   ★ 电机 PID 参数 + 限幅
│   │   ├── control_config.h                 ★ 控制 PID 参数 + 限幅
│   │   ├── rtos_config.h                    ★ 任务周期 + 模式开关
│   │   ├── pid.h                            PID 控制器
│   │   ├── PWM.h                            软件 PWM
│   │   ├── Motor.h                          直流电机驱动
│   │   ├── car_attitude.h                   姿态估计
│   │   ├── car_control.h                    运动控制
│   │   ├── IMU.h                            ICM-45686 姿态解算
│   │   ├── inv_imu_driver.h                 IMU 寄存器驱动
│   │   ├── inv_imu_transport.h              IMU SPI 传输层
│   │   ├── inv_imu_defs.h                   IMU 驱动宏定义
│   │   ├── inv_imu_version.h                IMU 驱动版本
│   │   ├── inv_imu.h                        IMU 驱动配置
│   │   ├── inv_imu_regmap_le.h              IMU 寄存器映射
│   │   ├── read_aux_data_mode.h             ★ IMU SPI read/write + 初始化接口
│   │   ├── delay.h                          ★ 微秒/毫秒延时（DWT 周期计数器）
│   │   ├── sys_time.h                       ★ 系统时间接口
│   │   ├── oled_spi_V0.2.h                  SSD1306 OLED 驱动
│   │   ├── oledfont.h                       OLED 字库数据
│   │   ├── oledpicture_V0.2.h               OLED 图片数据
│   │   ├── lcd_st7789.h                     ★ ST7789 LCD 驱动（从 spi.h 提取）
│   │   ├── lcd_model.h                      LCD 图形测试
│   │   ├── lcd_fonts.h                      LCD 字库数据
│   │   ├── lcd_image.h                      LCD 图片数据
│   │   ├── keyboard.h                       4×4 矩阵键盘
│   │   ├── MusicPlayer.h                   蜂鸣器音乐播放器
│   │   ├── MusicScore.h                    乐谱数据
│   │   ├── uart_fifo.h                     UART 帧收发协议
│   │   ├── arm.h                            机械臂 2 连杆运动学
│   │   ├── ARobot.h                         机械臂高层接口
│   │   ├── roboticArm.h                    机械臂舵机配置
│   │   ├── holder2D.h                       2D 激光云台
│   │   ├── w25q256.h                        ★ W25Q256 32MB Flash 低层驱动
│   │   ├── lfs_port.h                       ★ LittleFS 端口层
│   │   ├── lfs.h                            LittleFS v2.11.3 核心头文件
│   │   ├── lfs_util.h                       LittleFS 工具头文件
│   │   ├── FreeRTOS_CLI.h                   FreeRTOS+CLI v202212 头文件
│   │   ├── FreeRTOSConfig.h                 [CubeMX] FreeRTOS 内核配置
│   │   ├── stm32h7xx_hal_conf.h             [CubeMX] HAL 模块使能
│   │   ├── stm32h7xx_it.h                   [CubeMX] 中断声明
│   │   ├── main.h                           [CubeMX] Pin 宏定义 + Error_Handler
│   │   ├── gpio.h / tim.h / usart.h         [CubeMX] 外设初始化声明
│   │   ├── spi.h / quadspi.h / usb_otg.h    [CubeMX] 外设初始化声明
│   │   ├── memorymap.h / adc.h / i2c.h/...  [CubeMX] 外设初始化声明
│   │   └── USB_DEVICE/App/usbd_cdc_if.h     [CubeMX] USB CDC 接口
│   │
│   └── Src/                                # 源文件
│       ├── main.c                           [CubeMX] 程序入口 + USER CODE
│       ├── freertos.c                       [CubeMX] RTOS 任务创建 + Handler
│       ├── stm32h7xx_it.c                   [CubeMX] 中断服务程序
│       ├── stm32h7xx_hal_msp.c              [CubeMX] HAL MSP 初始化
│       ├── stm32h7xx_hal_timebase_tim.c     [CubeMX] TIM7 HAL 时基
│       ├── system_stm32h7xx.c               [CubeMX] 系统初始化
│       ├── gpio.c / tim.c / usart.c         [CubeMX] 外设初始化
│       ├── spi.c / quadspi.c / ...          [CubeMX] 外设初始化
│       │
│       ├── app_init.c                       ★ AppInit_Task 一次性初始化
│       ├── app_control.c                    ★ AppCarControl_Task 10ms 控制
│       ├── app_sensor.c                     ★ AppIMUService_Task 5ms IMU
│       ├── app_device.c                     ★ 显示/广播/UART/键盘/蜂鸣器任务
│       ├── app_sysmon.c                     ★ 系统监控（堆/栈水位）
│       ├── app_console.c                    ★ USB CDC CLI 控制台
│       ├── app_qspi.c                       ★ QSPI Flash + LittleFS 服务
│       ├── app_rtos_hooks.c                 ★ FreeRTOS 钩子 + 运行时统计
│       ├── display_service.c                ★ 显示抽象（编译期切换 OLED/LCD）
│       ├── pid.c                            PID 实现
│       ├── PWM.c                            软件 PWM 实现
│       ├── Motor.c                          电机控制实现
│       ├── car_attitude.c                   姿态估计实现
│       ├── car_control.c                    运动控制实现
│       ├── IMU.c                            Madgwick AHRS 算法
│       ├── inv_imu_driver.c                 IMU 寄存器驱动实现
│       ├── inv_imu_transport.c              IMU 传输层实现
│       ├── read_aux_data_mode.c             IMU SPI 接口 + 初始化
│       ├── delay.c                          ★ DWT 微秒/毫秒延时
│       ├── sys_time.c                       ★ 系统时间实现
│       ├── oled_spi_V0.2.c                 SSD1306 OLED 驱动
│       ├── lcd_st7789.c                     ★ ST7789 LCD 驱动（从 spi.c 提取）
│       ├── lcd_model.c                      LCD 图形测试
│       ├── lcd_fonts.c                      LCD 字库
│       ├── lcd_image.c                      LCD 图片
│       ├── keyboard.c                       矩阵键盘扫描
│       ├── MusicPlayer.c                   蜂鸣器音乐播放
│       ├── uart_fifo.c                     UART 帧接收
│       ├── arm.c                            机械臂运动学实现
│       ├── ARobot.c                         机械臂高层接口实现
│       ├── roboticArm.c                    机械臂舵机控制
│       ├── holder2D.c                       2D 激光云台
│       ├── w25q256.c                        ★ W25Q256 低层驱动
│       ├── lfs_port.c                       ★ LittleFS 端口实现
│       ├── lfs.c                            LittleFS v2.11.3 核心
│       ├── lfs_util.c                       LittleFS 工具函数
│       ├── FreeRTOS_CLI.c                   FreeRTOS+CLI 命令解析器
│       └── USB_DEVICE/App/usbd_cdc_if.c     [CubeMX] USB CDC 接口实现
│
├── Drivers/
│   ├── CMSIS/                               ARM CMSIS Core + DSP + RTOS2
│   └── STM32H7xx_HAL_Driver/               STM32 HAL 库
│
├── Middlewares/
│   ├── ST/STM32_USB_Device_Library/         USB Device 库 + CDC 类
│   └── Third_Party/FreeRTOS/Source/         FreeRTOS v10.3.1 内核
│
├── SCSLib/                                  第三方 SCS 串行舵机协议库
│
├── USB_DEVICE/                              USB 设备配置
│   ├── App/usb_device.c, usbd_cdc_if.c
│   └── Target/usbd_conf.c
│
├── MDK-ARM/                                 Keil MDK 项目文件
│   ├── STMH750VBT6_Celebright_V3.uvprojx
│   └── startup_stm32h750xx.s
│
└── examples/                                SCSLib 舵机示例
```

> `[CubeMX]` = STM32CubeMX 生成，修改限 `USER CODE BEGIN/END`  
> `★` = 重构新增或重要变更的文件

---

## 3. 模块规格

### 3.1 Common — 公共基础

#### 3.1.1 config.h / platform_config.h / motor_config.h / control_config.h / rtos_config.h

**路径**: `Core/Inc/config.h` (+ 4 sub-files)

**用途**: 全局编译期常量定义

| 文件 | 内容 | 示例 |
|------|------|------|
| `platform_config.h` | 轮径 47mm, 轮距 66.25mm, 编码器 1061.268 脉冲/圈 | `WHEEL_DIR`, `FRAME_W_HALF`, `V_REAL_TO_ENC` |
| `motor_config.h` | 4 电机 PID P/I/D 值 + 限幅 | `P_LF=0.02`, `I_LF=0.01`, `D_LF=0.015` |
| `control_config.h` | 位置/角度/角速度 PID + 限幅 | `P_POS=2.0`, `P_SPIN=2.0`, `P_V_ANGLE=1.0` |
| `rtos_config.h` | 任务周期 + 模式开关 | `TASK_ITV_CAR=10ms`, `USE_CAR_CONTROL=1`, `V_ANGLE_PID=1` |

#### 3.1.2 delay.c / delay.h

**路径**: `Core/Src/delay.c`, `Core/Inc/delay.h`

**用途**: 微秒/毫秒延时。使用 Cortex-M7 **DWT->CYCCNT** 周期计数器（480MHz CPU 时钟），不碰 SysTick。

**API**:
```c
void delay_init(uint32_t cpu_freq_mhz);  // 设置 CPU 频率 + 使能 DWT + 配 SysTick 时钟源
void delay_us(uint32_t nus);             // 微秒延时（spin-wait on DWT->CYCCNT）
void delay_ms(uint32_t nms);             // 毫秒延时（loop delay_us × 1000）
```

**实现细节**:
- `delay_init(480)`: 调用 `HAL_SYSTICK_CLKSourceConfig(HCLK)` 确保 SysTick 运行在 240MHz，然后使能 DWT 周期计数器
- `delay_us(n)`: 计算目标 ticks = n × 480 (MHz)，spin-wait 直到 DWT->CYCCNT 差值达到目标
- `delay_ms(n)`: 循环调用 `delay_us(1000)` n 次

**依赖**: `main.h` (DWT 寄存器, HAL_SYSTICK_CLKSourceConfig)

#### 3.1.3 sys_time.c / sys_time.h

**路径**: `Core/Src/sys_time.c`, `Core/Inc/sys_time.h`

**用途**: 系统时间接口。`nowtime` 在此定义（原在 main.c），`IMU.c` 通过此头文件访问。

**API**:
```c
extern uint32_t nowtime;                  // 100us 计数值
uint32_t SysTime_GetMs(void);             // 毫秒计数 (HAL_GetTick)
uint32_t SysTime_GetUs(void);             // 微秒计数 (DWT->CYCCNT)
```

### 3.2 Control — 控制层

#### 3.2.1 pid.c / pid.h

**路径**: `Core/Src/pid.c`, `Core/Inc/pid.h`

**用途**: 增量式 + 位置式 PID 控制器。独立模块，无外部依赖。

**API**:
```c
void  Set_PID(pid *obj, float p, float i, float d);                            // 设置 PID 参数
void  Set_PID_Limit(pid *obj, float lim_inc, float lim_pos, float lim_int);    // 设置限幅
float PID_Cal_Inc(pid *obj, float current, float target);                       // 增量式 PID
float PID_Cal_Pos(pid *obj, float current, float target);                       // 位置式 PID
void  PID_Clear(pid *obj);                                                      // 清除积分 + 历史误差
```

**数据结构**:
```c
typedef struct __pid {
    float p, i, d;                   // PID 系数
    float output_limit_inc;          // 增量输出限幅
    float output_limit_pos;          // 位置输出限幅
    float integral_error_limit;      // 积分限幅（抗积分饱和）
    float Output;                    // 当前输出
    float LastError, PrevError;      // 前 2 次误差（增量式）
    float IntegralError;             // 积分累加（位置式）
} pid;
```

#### 3.2.2 PWM.c / PWM.h

**路径**: `Core/Src/PWM.c`, `Core/Inc/PWM.h`

**用途**: 4 通道软件 PWM。通过 TIM6 ISR (40kHz) 生成 50Hz PWM。

**全局变量**: `dutyA, dutyB, dutyC, dutyD` (uint16_t, 0-500, 对应 0%-100%占空比)

**API**: `SetPWM(uint8_t channel, uint16_t duty)`

**实现**: TIM6 以 40kHz 频率触发中断。ISR 在 `TIM6_DAC_IRQHandler` 中维护计数器 `pwm_counter` (500→0)，每个通道在 `pwm_counter < dutyX` 时 GPIO 置高。

#### 3.2.3 Motor.c / Motor.h

**路径**: `Core/Src/Motor.c`, `Core/Inc/Motor.h`

**用途**: 4 路直流电机驱动。方向控制 + 编码器速度闭环 + PWM 输出。

**数据结构**:
```c
typedef struct __motor {
    GPIO_TypeDef *motor_gpio1, *motor_gpio2;  // 方向引脚
    uint16_t motor_pin1, motor_pin2;
    TIM_HandleTypeDef *htim;                   // PWM 定时器
    _Bool dir;                                 // 方向标志
    pid v_pid;                                 // 增量式 PID 实例
    int EncSource, total_enc;                  // 编码器值
    float v_enc, v_real, target_v_enc, duty;   // 速度 + 占空比
} motor;
```

**全局实例**: `motor_LeftFront, motor_LeftRear, motor_RightFront, motor_RightRear`

**API**:
```c
void  init_motor(void);                                                      // 初始化 4 电机
void  Motor_Update_Input_All(void);                                          // 读取编码器 → 计算速度
void  Motor_Update_Output_All(void);                                         // PID 计算 → 输出 PWM
void  Wheel_Set_V_Real(float v_left, float v_right);                        // 设置左右轮目标速度 (mm/s)
float Wheel_Get_V_Real(_motor_choice LEFT/RIGHT);                           // 获取单侧速度
float Wheel_Get_Distance(_motor_choice LEFT/RIGHT);                         // 获取累计里程 (mm)
void  Wheel_Clear_Distance(void);                                            // 清零里程
```

**控制算法**: 编码器计数 → 增量式 PID (比例+积分+微分) → 占空比 → 软件 PWM 输出

#### 3.2.4 car_attitude.c / car_attitude.h

**路径**: `Core/Src/car_attitude.c`, `Core/Inc/car_attitude.h`

**用途**: 小车姿态估计与角速度控制。

**数据结构**:
```c
typedef struct __car_attitude {
    float current_v_line, target_v_line;   // 线速度 mm/s
    float current_v_angle, target_v_angle; // 角速度 °/s
    _Bool flag_stop, updated;              // 状态标志
    pid pid_v_angle;                       // 角速度 PID
} _car_attitude;
```

**共享状态** `car_state_t` (在 app.h 中定义，**消除循环依赖**):
```c
typedef struct {
    float yaw, yaw_all;       // 偏航角
    int yaw_circles;          // 偏航圈数
    float v_line, v_angle;    // 当前速度
    float v_line_target, v_angle_target;  // 目标速度
    _Bool flag_stop;
} car_state_t;
```

**API**:
```c
void init_Car_Attitude(void);                                              // 初始化角速度 PID
void Car_Attitude_Update_Input(void);                                      // 编码器 → 线速度/角速度
void Car_Attitude_Update_Output(void);                                     // 角速度 PID → 左右轮速度
void Set_Car_Attitude(float v_line_target, float v_angle_target);         // 设置目标姿态
void Car_Attitude_Yaw_Update(float v_angle, float time);                  // IMU 积分更新偏航角
void Set_Car_Stop/Start(void);                                             // 停车/启动
```

**更新周期**: 10ms (CarControl_Handler)

#### 3.2.5 car_control.c / car_control.h

**路径**: `Core/Src/car_control.c`, `Core/Inc/car_control.h`

**用途**: 高层运动指令——走直线/弧线/旋转/停止。

**运动模式**:
| 模式 | 参数 | 说明 |
|------|------|------|
| `GO_LINE` | x=距离(mm) | 位置 PID 控制速度，走直线 |
| `TO_POINT` | x, y=目标点(mm) | 两轮差速走圆弧 |
| `SPIN` | angle=角度(°) | 原地旋转 |
| `STOP` | — | 停车 |

**API**:
```c
void init_Car_Contorl(void);                                              // 初始化位置 PID
void Set_Car_Control(float x, float y, float angle);                     // 设定运动目标
void Car_Control_Update_Input(void);                                      // 检查进度（位置/角度）
void Car_Control_Update_Output(void);                                     // 位置 PID → 调用 Set_Car_Attitude
```

**更新周期**: 10ms (CarControl_Handler)

### 3.3 Sensor — 传感器层

#### 3.3.1 IMU.c / IMU.h

**路径**: `Core/Src/IMU.c`, `Core/Inc/IMU.h`

**用途**: ICM-45686 9 轴 IMU 的 Madgwick AHRS 姿态解算。

**API**:
```c
void IMU_init(void);                          // 初始化 IMU + 四元数 + 陀螺仪零偏校准
void IMU_getYawPitchRoll(float *ypr);         // 获取 yaw(0)/pitch(1)/roll(2) 度
void IMU_TT_getgyro(float *motion6);          // 获取陀螺仪 3 轴 + 加速度 3 轴
```

**输出变量**: `imu_g_z` (z 轴角速度 °/s), `ypr[3]`, `motion6[7]`

**更新周期**: 5ms (IMUService_Handler)

#### 3.3.2 inv_imu_driver / inv_imu_transport / read_aux_data_mode

**用途**: InvenSense ICM-45686 寄存器级驱动 + SPI4 传输层。

| 文件 | 用途 |
|------|------|
| `inv_imu_driver.c/h` | 寄存器配置读写、FIFO、中断 |
| `inv_imu_transport.c/h` | 抽象传输层 (read_reg/write_reg/sleep_us) |
| `read_aux_data_mode.c` | SPI4 具体实现: `icm45686_read_regs/write_regs` |

### 3.4 Device — 设备层

#### 3.4.1 display_service.c / display_service.h

**用途**: OLED/LCD 显示抽象。编译期 `#if` 切换后端。

**API**:
```c
void DisplayService_Init(void);
void DisplayService_Clear(void);
void DisplayService_ShowText(uint16_t x, uint16_t y, const char *text);
void DisplayService_ShowNumber(uint16_t x, uint16_t y, int32_t num, uint8_t len);
```

**切换方式**: `app.h` 中 `#define APP_DISPLAY_TYPE DISPLAY_TYPE_OLED` 或 `DISPLAY_TYPE_LCD`

#### 3.4.2 oled_spi_V0.2.c/h — OLED 驱动

128×64 SSD1306 OLED，SPI2 接口。字符、汉字、BMP 位图。

#### 3.4.3 lcd_st7789.c/h — LCD 驱动

ST7789 240×280 TFT LCD，SPI2 接口。从 `spi.c` 提取。API：`SPI_LCD_Init`, `LCD_Clear`, `LCD_DisplayString`, `LCD_DisplayNumber` 等。

#### 3.4.4 keyboard.c/h — 矩阵键盘

4×4 矩阵键盘扫描。`Key_StartScanTask()` 创建内部扫描任务，通过 `queue_key` 队列传递按键。

#### 3.4.5 MusicPlayer.c/h — 音乐播放

TIM13 PWM 驱动蜂鸣器。预置 6 首乐谱（Sakura/MEGALOVANIA 等）。默认禁用 (`APP_ENABLE_MUSIC=0`)。

#### 3.4.6 uart_fifo.c/h — UART 帧通信

帧格式 `0xAA + DATA + 0x55`。中断接收 → FreeRTOS 队列 → 任务处理。

### 3.5 Robot — 机械臂

#### 3.5.1 arm.c/h — 平面 2 连杆运动学

**路径**: `Core/Src/arm.c`, `Core/Inc/arm.h`

**用途**: 4 自由度平面机械臂的正向/逆向运动学。

**API**:
```c
uint8_t _initRoboticArm(RoboticArm *arm, uint16_t H, uint16_t L1, uint16_t L2, uint16_t L3, ...);
Vf_Position _forwardKinematics(const RoboticArm *arm, const V_Angle *angle);
uint8_t _inverseKinematics(RoboticArm *arm, const V_Position *target, V_Angle *angle);
float _radiansToDegrees / _degreesToRadians(float);
```

**数据结构**:
```c
typedef struct {
    uint16_t H, L1, L2, L3;    // (已重命名: L→link 避免与 MusicPlayer 音符宏冲突)
    float angle, theta1-3;
    ArmThirdAngleType theta3_type;
    ...
} RoboticArm;
```

#### 3.5.2 roboticArm.c/h — 舵机控制

SCS 串行舵机 (ID: 1, 2, 88, 9, 66) 角度限位 + 初始化。

#### 3.5.3 ARobot.c/h — 机械臂高层接口

`initArm()`, `moveArm(x, y, z)` — 运动学 + 舵机控制。

### 3.6 Storage — 存储

#### 3.6.1 w25q256.c/h — Flash 低层驱动

W25Q256 32MB QSPI NOR Flash 驱动。Device ID: 0x4019, 容量: 33554432 bytes。

**API**: `W25Q256_Init/Read/Write/EraseSector/EraseBlock/ChipErase`

#### 3.6.2 lfs_port.c/h — LittleFS 端口

将 LittleFS 块操作映射到 W25Q256。**16MB 分区** (4096 blocks × 4KB)。

#### 3.6.3 app_qspi.c/h — QSPI 服务层

`AppQSPI_Mount/Format/ListDir/ReadFile/BuildInfo` 等高层 API。

### 3.7 System — 系统管理

#### 3.7.1 app_sysmon.c/h — 系统监控

**用途**: 每 1s 采集堆剩余 + 任务栈水位，每 200ms 更新 OLED 显示。

**显示内容**: `Heap:32640B`, `D:120 C:115` (任务栈剩余水位)

#### 3.7.2 app_console.c/h — CLI 控制台

**用途**: USB CDC 虚拟串口命令行。FreeRTOS+CLI 解析器。

**命令**:
| 命令 | 功能 |
|------|------|
| `help` | 命令列表 |
| `tasks` | FreeRTOS 任务状态 |
| `stats` | 运行时 CPU 占比 |
| `sys` | 堆剩余 + 栈水位 |
| `flash mount/ls/info/format` | QSPI Flash 操作 |
| `car 1000 0` / `car 0 180` | 小车直行/旋转 |

**数据流**: USB CDC RX → StreamBuffer → Console Task → CDC_Transmit_FS TX

#### 3.7.3 app_rtos_hooks.c/h — FreeRTOS 钩子

**用途**: 运行时统计 (DWT->CYCCNT 代替 TIM2) + 栈溢出/内存失败钩子。

---

## 4. FreeRTOS 任务表

| Task Name | Handler | App Function | Priority | Stack | Period | File |
|-----------|---------|-------------|----------|-------|--------|------|
| LEDBlink | LedBlink_Handler | (inline) | AboveNormal(32) | 128w | 1s | freertos.c |
| Broadcast | Broadcast_Handler | AppBroadcast_Task | Normal1(25) | 256w | 50ms | app_device.c |
| Uart4Rx | Uart4Rx_Handler | AppUart4Rx_Task | Normal7(31) | 256w | block | app_device.c |
| IMUService | IMUService_Handler | AppIMUService_Task | Normal3(27) | 128w | 5ms | app_sensor.c |
| CarControl | CarControl_Handler | AppCarControl_Task | Normal5(29) | 128w | 10ms | app_control.c |
| KeyScan | KeyScan_Handler | AppKeyScan_Task | Normal6(30) | 128w | idle | app_device.c |
| Buzzer | Buzzer_Handler | AppBuzzer_Task | Normal1(25) | 128w | idle | app_device.c |
| Display | Display_Handler | AppDisplay_Task | Normal1(25) | 128w | 100ms | app_device.c |
| Reserved | Reserved_Handler | AppReserved_Task | Normal1(25) | 1024w | 50ms | app_device.c |
| Uart3Rx | Uart3Rx_Handler | AppUart3Rx_Task | Normal7(31) | 256w | block | app_device.c |
| SysMon | SysMon_Handler | AppSysMon_Task | Low(8) | 512w | 200ms | app_sysmon.c |
| Console | Console_Handler | AppConsole_Task | Normal1(25) | 512w | block | app_console.c |
| AppInit | (xTaskCreate) | AppInit_Task | Normal5(29) | 512w | once | app_init.c |

---

## 5. FreeRTOS 对象

| 对象 | 类型 | 大小 | 用途 |
|------|------|------|------|
| `queue_key` | CMSIS MessageQ | 1×char | 按键传递 |
| `uart3_frame_queue` | Queue | 5×64B | UART3 帧缓冲 |
| `uart4_frame_queue` | Queue | 5×64B | UART4 帧缓冲 |
| `console_rx_stream` | StreamBuffer | 256B | USB CDC RX |
| `semphr_buzzer_trigger` | BinarySem | — | 蜂鸣器 |
| `semphr_uart_receive` | CountingSem | max=16 | UART 接收 |

---

## 6. 中断表

| IRQ | Priority | Handler | Function |
|-----|----------|---------|----------|
| SysTick | 15 | SysTick_Handler | HAL_IncTick + xPortSysTickHandler |
| TIM6_DAC | 5 | TIM6_DAC_IRQHandler | 软件 PWM (40kHz) |
| TIM7 | 5 | TIM7_IRQHandler | HAL 时基 (1kHz) |
| UART4 | 5 | UART4_IRQHandler | 帧接收 |
| USART3 | 5 | USART3_IRQHandler | 帧接收 |
| PendSV | 15 | PendSV_Handler | FreeRTOS 上下文切换 |

---

## 7. 内存布局

| 区域 | 大小/地址 | 用途 |
|------|----------|------|
| Flash (内部) | 128KB (0x08000000) | 程序代码 |
| Flash (W25Q256 QSPI) | 32MB (0x90000000) | 文件系统 (16MB LittleFS) |
| RAM (DTCM) | 128KB (0x20000000) | 堆 + 栈 |
| RAM (AXI SRAM) | 512KB (0x24000000) | 一般数据 |
| FreeRTOS Heap | 81920 bytes | 动态分配 (heap_4) |
| LittleFS LFS config | 16MB / 4096 blocks | W25Q256 文件系统 |

---

## 8. 模块剪裁开关

在 `app.h` 中修改 `APP_ENABLE_*` 宏为 0 即可禁用模块：

| 开关 | 默认值 | 效果 |
|------|--------|------|
| `APP_ENABLE_SYSMON` | 1 | 系统监控任务 + OLED 显示 |
| `APP_ENABLE_CONSOLE` | 1 | USB CDC CLI 控制台 |
| `APP_ENABLE_QSPI` | 1 | W25Q256 + LittleFS |
| `APP_ENABLE_DISPLAY` | 1 | OLED/LCD 显示 |
| `APP_ENABLE_KEYBOARD` | 1 | 4×4 矩阵键盘 |
| `APP_ENABLE_MUSIC` | 0 | 蜂鸣器音乐 |
| `APP_ENABLE_UART_FIFO` | 1 | UART 帧通信 |
| `APP_ENABLE_ROBOTIC_ARM` | 0 | 机械臂子系统 |
| `APP_ENABLE_IMU` | 1 | IMU 传感器 |
| `APP_ENABLE_LEDBLINK` | 1 | LED 闪烁 |
| `APP_ENABLE_BROADCAST` | 0 | 蓝牙广播 |
| `APP_DISPLAY_TYPE` | OLED | OLED 或 LCD 切换 |

---

## 9. 构建与调试

### 9.1 构建

1. Keil MDK V5.32+ 打开 `MDK-ARM/STMH750VBT6_Celebright_V3.uvprojx`
2. Build (F7)
3. Flash Download (F8)

### 9.2 CubeMX 再生后恢复

| 步骤 | 操作 |
|------|------|
| 1 | MDK 项目 include path 恢复 `../SCSLib` |
| 2 | MDK 项目添加 `sys_time.c`, `app_init.c`, `app_control.c`, `app_sensor.c`, `app_device.c`, `app_console.c`, `app_sysmon.c`, `app_rtos_hooks.c`, `app_qspi.c`, `display_service.c`, `w25q256.c`, `lfs_port.c`, `lcd_st7789.c`, `FreeRTOS_CLI.c`, `lfs.c`, `lfs_util.c` |
| 3 | `freertos.c` Handler body 恢复一行 `AppXxx_Task(argument)` |

### 9.3 常见问题

**Q: `osKernelStart()` 后系统挂起？**  
A: 检查 CubeMX USB_DEVICE 配置。CubeMX 可能在 `LedBlink_Handler` 中自动插入 `MX_USB_DEVICE_Init()`。在 CubeMX → USB_DEVICE → Platform Settings 中设置初始化任务为 "none"，USB 初始化由 `AppInit_Task` 处理。

**Q: `lcd_st7789.h: error #169`**  
A: 文件损坏。重新生成 lcd_st7789.h 或从备份恢复。

**Q: `configCPU_CLOCK_HZ` 不匹配？**  
A: STM32H750 SysTick 时钟 = HCLK (240MHz)。FreeRTOSConfig.h 中已修正为 `SystemCoreClock / 2U`。

---

## 10. CubeMX 配置参考

### 关键设置

| 类别 | 设置 | 值 |
|------|------|------|
| System Core → RCC | HSE | Crystal/Ceramic Resonator |
| System Core → SYS | Timebase Source | TIM7 |
| Connectivity → USB_OTG_FS | Mode | Device Only |
| Middleware → USB_DEVICE | Class | CDC (Virtual Port Com) |
| Middleware → FREERTOS | Interface | CMSIS_V2 |
| Middleware → FREERTOS | Memory | Static Allocation |
| Middleware → FREERTOS | TOTAL_HEAP_SIZE | 81920 |

---

*Version: v3.1 | Updated: 2026-05-09*
