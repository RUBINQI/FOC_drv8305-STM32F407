# FOC_stm32f4_as5600_drv8305 工程技术手册

## 1. 工程简介
本工程基于 STM32F407VETx MCU，实现了一个高性能的无刷电机（PMSM/BLDC）磁场定向控制（FOC）系统。
当前版本运行为 **电流环闭环**，保留速度环代码但已暂停调用，具备完善的工程架构和非阻塞通信机制。

### 核心特性
*   **控制策略**: 纯电流环闭环。
*   **采样方式**: 三电阻低侧采样 (3-Shunt Low-side Sensing)。
*   **控制频率**: 20kHz (FOC 环路) + 1kHz (应用) + 200Hz (通信环)。
*   **传感器**: AS5600 磁编码器 (I2C DMA 非阻塞读取)。
*   **驱动器**: TI DRV8305 (SPI 配置 + 硬件 DC 校准)。

---

## 2. 硬件架构与引脚配置

| 功能模块 | 硬件外设 | 引脚配置 | 备注 |
| :--- | :--- | :--- | :--- |
| **MCU** | STM32F407VETx | - | 168MHz 主频 |
| **PWM 输出** | TIM1 (CH1/2/3) | PE9, PE11, PE13 | 中心对齐模式，20kHz |
| **ADC 触发** | TIM1 TRGO | - | Update 事件触发注入组转换 |
| **电流采样** | ADC1 (Injected) | PA3, PA4, PA5 | 对应 A, B, C 三相电流 |
| **栅极驱动** | SPI1 | PA5(CLK), PA6(MISO), PA7(MOSI) | DRV8305 配置与状态读取 |
| **角度传感器** | I2C1 (DMA) | PB6(SCL), PB7(SDA) | AS5600 角度读取 |
| **调试通信** | USART1 (DMA) | PA9(TX), PA10(RX) | VOFA+ 波形监控 |

---

## 3. 软件架构设计

系统采用 **分层多速率架构 (Multi-Rate Architecture)**，确保实时性与任务解耦。

### 3.1 快速环路 (Fast Loop) - 20kHz
*   **触发源**: ADC 注入组转换完成中断 (`HAL_ADCEx_InjectedConvCpltCallback`)。
*   **优先级**: 最高 (NVIC Priority 0)。
*   **任务**:
    1.  **电流采样**: 读取 ADC 原始值并转换为物理电流 (Amps)。
    2.  **角度计算**: 计算电角度与三角函数。
    3.  **FOC 核心算法**: Clarke -> Park -> PI 控制 -> InvPark -> SVPWM。
    4.  **PWM 更新**: 写入 TIM1 CCR 寄存器。

### 3.2 中速环路 (Medium Loop) - 1kHz
*   **触发源**: TIM9 更新中断 (`HAL_TIM_PeriodElapsedCallback`)。
*   **优先级**: 次高 (NVIC Priority 1)。
*   **任务**:
    1.  **角度更新触发**: 启动下一次 AS5600 的 DMA 读取 (`AS5600_ReadAngleDMA`)，为快速环路提供机械角度数据。
    2.  **备注**: 此环路目前仅负责传感器数据更新，不执行任何闭环控制逻辑。

### 3.3 慢速环路 (Slow Loop) - 200Hz
*   **触发源**: 主循环 `while(1)` + `HAL_GetTick()`。
*   **优先级**: 最低 (背景任务)。
*   **任务**:
    1.  **数据监控**: 通过 DMA 发送 VOFA+ 协议数据包 (目标转速, 实际转速, Iq, I_mag)。
    2.  **人机交互**: LED 状态指示、按键扫描。
    3.  **故障保护**: 温度监控、长时间堵转保护。

---

## 4. FOC 核心算法流程详解

文件位置: `Core/Src/foc.c`

算法流水线严格遵循以下 8 个步骤：

1.  **电流物理量转换**:
    *   公式: $I_{phase} = (ADC_{raw} - Offset) \times K_{curr}$
    *   $K_{curr}$ 系数包含: 3.3V 参考电压, 12-bit 分辨率, 80V/V 运放增益, 50mΩ 采样电阻。
    *   *优化*: 利用 Kirchhoff 定律 ($I_a + I_b + I_c = 0$) 校验采样误差。

2.  **角度归一化**:
    *   将机械角度转换为电角度，并归一化至 $[0, 2\pi]$。
    *   使用 `arm_sin_f32` / `arm_cos_f32` 计算正余弦。

3.  **Clarke 变换 (3s -> 2s)**:
    *   输入: $I_a, I_b$
    *   输出: $I_\alpha, I_\beta$

4.  **Park 变换 (2s -> 2r)**:
    *   输入: $I_\alpha, I_\beta, \sin\theta, \cos\theta$
    *   输出: $I_d, I_q$ (旋转坐标系下的直流分量)

5.  **PI 控制器**:
    *   **D 轴**: 目标 $I_d = 0$ (MTPA 控制)。
    *   **Q 轴**: 目标 $I_q$ (由速度环 PI 输出或用户指定)。
    *   输出: $V_d, V_q$ (电压指令)。

6.  **反 Park 变换 (2r -> 2s)**:
    *   输入: $V_d, V_q$
    *   输出: $V_\alpha, V_\beta$

7.  **SVPWM 生成 (2s -> 3s)**:
    *   输入: $V_\alpha, V_\beta$
    *   算法: 标准空间矢量调制 (Space Vector Modulation)。
    *   **代码优化**:
        ```c
        // 预定义常数提高可读性
        #define ONE_BY_SQRT3  0.5773503f
        #define SQRT3_BY_2    0.8660254f
        
        // 逆 Clarke + 归一化处理
        float Va = Valpha * ONE_BY_SQRT3;
        float Vb = (-0.5f * Valpha + SQRT3_BY_2 * Vbeta) * ONE_BY_SQRT3;
        float Vc = (-0.5f * Valpha - SQRT3_BY_2 * Vbeta) * ONE_BY_SQRT3;
        ```

8.  **PWM 输出**:
    *   中心对齐模式，CCR 值更新。

---

## 5. 调试与参数整定指南

### 5.1 扭矩控制接口
在 `main.c` 或调试器中修改全局变量：
```c
// 设置目标扭矩 (单位: Nm)
target_torque_nm = 0.05f; 
// 内部会自动换算为目标电流 Iq_target
```

### 5.2 PI 参数整定 (电流环)
1.  **准备**: 将 D/Q 轴 $K_i$ 设为 0，仅调节 $K_p$。
2.  **Kp 整定**:
    *   给定阶跃信号 (如 Iq 目标从 0.1A 跳变至 0.4A)。
    *   增大 $K_p$ 直到电流响应迅速且无明显震荡 (通常在 0.05 ~ 0.20 之间)。
3.  **Ki 整定**:
    *   逐步增大 $K_i$ 消除稳态误差 (通常在 100 ~ 500 之间)。
    *   注意防止积分过冲。

### 5.3 电机参数配置
**目标电机**: 2208-80T 云台电机 (高阻抗、大电感、低KV)

*   **极对数 (POLE_PAIRS)**: 7
*   **KV 值 (MOTOR_KV)**: 100.0f
*   **相电阻**: 8.25Ω
*   **相电感**: 4.25mH
*   **推荐 PI 参数**:
    *   **电流环**: Kp = 1.0, Ki = 200.0 (因为电感大，Kp 必须比航模电机大很多)

---

*文档版本: v2.2*
*最后更新: 2026-03-11*
