# 多陀螺仪系统设计与实现方案 (DMA + UART 空闲中断)

本方案设计旨在利用 STM32F407ZGT6 开发板，通过 **DMA 接收 + 串口空闲中断 (UART IDLE)** 的高效异步方式，同时读取并解析多达 6 个维特智能（Wit-Motion）串行陀螺仪的数据，解决官方 SDK 无法支持多实例的问题，并确保系统的高实时性、零丢包以及低 CPU 占用。

---

## 1. 项目背景与设计目标

### 1.1 现有局限性
维特智能官方 SDK 依赖全局变量数组 `sReg` 存储传感器寄存器数据。当接入多个串口陀螺仪时，各个串口接收的数据会直接写入同一个 `sReg` 中，导致数据发生覆盖和冲突。

### 1.2 设计目标
* **高拓展性**：支持 1 到 6 个陀螺仪同时工作，仅需修改通道宏定义和引脚绑定即可完成拓展。
* **低 CPU 开销**：采用 DMA 硬件搬运，配合串口空闲中断（一包数据传输完成后仅中断 CPU 一次），CPU 占有率控制在 1% 以下。
* **高可靠性**：软件具备 DMA 异常挂死自愈能力，能自动应对静电、热插拔等带来的物理干扰。

---

## 2. 硬件连接方案设计

### 2.1 物理分线与共地规范
由于 6 个陀螺仪的供电（VCC 3.3V）和地线（GND）接口较多，单片机排针无法提供足够的接口，硬件上必须采用**并联分线方式**：

1. **电源汇集端子 (VCC)**：将单片机的 1 个 `3.3V` 引脚引出，通过一个“一分六”的万能接线端子，并联分配给 6 个陀螺仪的 VCC。
2. **地线汇集端子 (GND)**：将单片机的 1 个 `GND` 引脚引出，通过另一个“一分六”的万能接线端子，并联分配给 6 个陀螺仪的 GND。**所有设备必须共地**，否则无法正常进行串口通信。

### 2.2 串口引脚分配表 (STM32F407ZGT6)
根据 STM32F407 的引脚映射，我们规划 6 路串口的物理引脚分配如下（完全避开了引脚复用冲突）：

| 陀螺仪编号 | 物理串口 | TX 引脚 | RX 引脚 | 串口用途 |
| :--- | :--- | :--- | :--- | :--- |
| **调试输出** | **USART1** | PA9 | PA10 | 连接 PC 串口助手（COM9），进行打印 |
| **Gyro 1** | **USART2** | PA2 | PA3 | 接收第一路陀螺仪数据 |
| **Gyro 2** | **USART3** | PB10 | PB11 | 接收第二路陀螺仪数据 (方案A选用) |
| **Gyro 3** | **UART4** | PC10 | PC11 | 接收第三路陀螺仪数据 |
| **Gyro 4** | **UART5** | PC12 | PD2 | 接收第四路陀螺仪数据 |
| **Gyro 5** | **USART6** | PC6 | PC7 | 接收第五路陀螺仪数据 |
| **Gyro 6** | 可复用引脚或通过模拟/外部串口 | - | - | 备用（可使用软件串口或 SPI/I2C 拓展） |

---

## 3. 维特智能协议解析与数据读取算法

维特智能传感器在“主动输出模式”下，通过串口向单片机发送 11 字节的数据包。

### 3.1 协议包结构 (11 字节)

| 字节序号 | 数据含义 | 具体说明 |
| :--- | :--- | :--- |
| **Byte 0** | 帧头 | 固定为 `0x55` |
| **Byte 1** | 包类型 | `0x51`: 加速度包；`0x52`: 角速度包；`0x53`: 角度包 |
| **Byte 2-3** | Data 0 (X轴) | 16位有符号数，小端模式（Byte 2 为低字节，Byte 3 为高字节） |
| **Byte 4-5** | Data 1 (Y轴) | 16位有符号数，小端模式（Byte 4 为低字节，Byte 5 为高字节） |
| **Byte 6-7** | Data 2 (Z轴) | 16位有符号数，小端模式（Byte 6 为低字节，Byte 7 为高字节） |
| **Byte 8-9** | Data 3 (温度) | 16位有符号数，小端模式 |
| **Byte 10** | 校验和 (Sum) | 校验和 = (Byte 0 + Byte 1 + ... + Byte 9) & 0xFF |

### 3.2 原始二进制到物理单位的转换算法

我们将接收到的低字节 `DataL` 和高字节 `DataH` 合成 16 位有符号整型 `int16_t`，然后套用以下官方公式进行单位转换：

#### 1. 加速度 (Acceleration)
* **计算公式**：
  $$a_x = \frac{\text{Data\_X}}{32768} \times 16g$$
* **C 语言解析代码**：
  `acc_x = (int16_t)((buf[3] << 8) | buf[2]) / 32768.0f * 16.0f;`

#### 2. 角速度 (Angular Velocity / Gyro)
* **计算公式**：
  $$w_x = \frac{\text{Data\_X}}{32768} \times 2000^\circ/s$$
* **C 语言解析代码**：
  `gyro_x = (int16_t)((buf[5] << 8) | buf[4]) / 32768.0f * 2000.0f;`

#### 3. 角度 (Angle)
* **计算公式**：
  $$\theta_x = \frac{\text{Data\_X}}{32768} \times 180^\circ$$
* **C 语言解析代码**：
  `angle_x = (int16_t)((buf[7] << 8) | buf[6]) / 32768.0f * 180.0f;`

---

## 4. 软件架构与核心代码框架

软件层采用“面向对象”设计，将传感器数据、DMA 缓冲区和串口句柄封装到独立的结构体中。

### 4.1 数据结构

```c
#define GYRO_COUNT  2  // 支持拓展到 6

// 陀螺仪物理测量数据
typedef struct {
    float acc[3];      // x, y, z 轴加速度 (g)
    float gyro[3];     // x, y, z 轴角速度 (°/s)
    float angle[3];    // Roll, Pitch, Yaw 角度 (°)
    float temp;        // 温度 (℃)
} GyroData_t;

// 陀螺仪接收与解析设备上下文
typedef struct {
    UART_HandleTypeDef *huart; // 绑定的物理串口句柄 (如 &huart2, &huart3)
    uint8_t rx_buf[64];        // DMA 接收环形缓存
    GyroData_t data;           // 解析后的物理值
    volatile uint8_t updated;  // 帧更新标志 (1: 有新数据)
} GyroDevice_t;

// 设备实体数组
extern GyroDevice_t g_gyros[GYRO_COUNT];
```

### 4.2 数据包流转解析函数

该函数处理 DMA 接收到的缓冲区，寻找 `0x55` 帧头，进行 Checksum 校验并转换物理单位，写入对应实例的结构体中：

```c
void ParseGyroBuffer(GyroDevice_t *device, uint16_t len) {
    uint16_t i = 0;
    
    // 因为是空闲中断触发，缓冲区里可能积累了多包数据（如加速度 + 角速度 + 角度 共 33 字节）
    while (i + 11 <= len) {
        // 1. 寻找帧头 0x55
        if (device->rx_buf[i] != 0x55) {
            i++;
            continue;
        }
        
        // 2. 校验和计算
        uint8_t sum = 0;
        for (uint16_t j = 0; j < 10; j++) {
            sum += device->rx_buf[i + j];
        }
        
        // 3. 校验成功，开始解析具体类型
        if (sum == device->rx_buf[i + 10]) {
            uint8_t *pkg = &(device->rx_buf[i]);
            uint8_t pkg_type = pkg[1];
            
            // 合成有符号 16 位整型
            int16_t d0 = (int16_t)((pkg[3] << 8) | pkg[2]);
            int16_t d1 = (int16_t)((pkg[5] << 8) | pkg[4]);
            int16_t d2 = (int16_t)((pkg[7] << 8) | pkg[6]);
            int16_t d3 = (int16_t)((pkg[9] << 8) | pkg[8]);
            
            switch (pkg_type) {
                case 0x51: // 加速度包
                    device->data.acc[0] = d0 / 32768.0f * 16.0f;
                    device->data.acc[1] = d1 / 32768.0f * 16.0f;
                    device->data.acc[2] = d2 / 32768.0f * 16.0f;
                    device->data.temp   = d3 / 100.0f;
                    break;
                    
                case 0x52: // 角速度包
                    device->data.gyro[0] = d0 / 32768.0f * 2000.0f;
                    device->data.gyro[1] = d1 / 32768.0f * 2000.0f;
                    device->data.gyro[2] = d2 / 32768.0f * 2000.0f;
                    break;
                    
                case 0x53: // 角度包
                    device->data.angle[0] = d0 / 32768.0f * 180.0f;
                    device->data.angle[1] = d1 / 32768.0f * 180.0f;
                    device->data.angle[2] = d2 / 32768.0f * 180.0f;
                    device->updated = 1; // 一整组数据包接收解析完成，触发更新标志
                    break;
                    
                default:
                    break;
            }
            i += 11; // 跨过当前成功解析的包
        } else {
            i++; // 校验失败，滑移一个字节继续寻找
        }
    }
}
```

### 4.3 底层中断事件分流

依靠 ST HAL 库的 `HAL_UARTEx_RxEventCallback`，将各个串口接收事件快速分发给对应的陀螺仪设备结构体：

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    for (int i = 0; i < GYRO_COUNT; i++) {
        if (huart->Instance == g_gyros[i].huart->Instance) {
            // 1. 调用解析函数
            ParseGyroBuffer(&g_gyros[i], Size);
            
            // 2. 重新开启当前通道的 DMA + 空闲中断接收，为下一包数据做准备
            HAL_UART_Ex_ReceiveToIdle_DMA(g_gyros[i].huart, g_gyros[i].rx_buf, 64);
            break;
        }
    }
}
```

---

## 5. DMA 故障自愈与系统初始化设计

### 5.1 动态初始化与串口映射列表 (一键配置设计)
为了使陀螺仪个数完全可配置，我们在软件中定义一个常量指针数组，依次列出所有可用的串口硬件句柄。这样，只需在宏定义中修改 `GYRO_COUNT`，初始化循环就会自动绑定相应数量的串口并开启 DMA，无需再手动修改多行初始化代码：

```c
// 1. 在 main.h 或 main.c 中定义可配置的陀螺仪个数
#define GYRO_COUNT  2  // 本期接入 2 个，未来可以直接修改为 3 ~ 5

// 2. 串口映射列表 (按顺序存放可用串口，必须与 CubeMX 中使能的串口一致)
UART_HandleTypeDef *const g_huart_list[5] = {
    &huart2,  // Gyro 1 (USART2)
    &huart3,  // Gyro 2 (USART3)
    &huart4,  // Gyro 3 (UART4)
    &huart5,  // Gyro 4 (UART5)
    &huart6   // Gyro 5 (USART6)
};

// 3. 一键初始化循环
void Gyro_System_Init(void) {
    for (int i = 0; i < GYRO_COUNT; i++) {
        g_gyros[i].huart = g_huart_list[i];
        g_gyros[i].updated = 0;
        memset(&(g_gyros[i].data), 0, sizeof(GyroData_t));
        
        // 开启当前通道的 DMA + 空闲中断接收
        HAL_UART_Ex_ReceiveToIdle_DMA(g_gyros[i].huart, g_gyros[i].rx_buf, sizeof(g_gyros[i].rx_buf));
    }
}
```

### 5.2 DMA 软件自愈机制
在实际工业现场或复杂的连线环境下，单片机容易受到静电、电磁干扰或者热插拔接线抖动的影响，导致 DMA 传输出现错误并挂死（停止接收）。
为了实现系统自愈，我们可以在主循环中增加一个软件检测机制，同样由 `GYRO_COUNT` 自动驱动：

```c
void CheckAndRecoverDMA(void) {
    static uint32_t last_check_time = 0;
    uint32_t now = HAL_GetTick();
    
    // 每隔 500ms 检查一次各个 DMA 通道的状态
    if (now - last_check_time > 500) {
        last_check_time = now;
        
        for (int i = 0; i < GYRO_COUNT; i++) {
            // 如果 500ms 内该通道没有更新数据，且 DMA 状态为 READY (说明 DMA 被异常终止了)
            if (g_gyros[i].huart->RxState == HAL_UART_STATE_READY) {
                // 执行自愈：强行重启 DMA 接收
                HAL_UART_AbortReceive(g_gyros[i].huart);
                HAL_UART_Ex_ReceiveToIdle_DMA(g_gyros[i].huart, g_gyros[i].rx_buf, sizeof(g_gyros[i].rx_buf));
                printf("Warning: DMA channel %d recovered from fault!\r\n", i);
            }
        }
    }
}
```

---

## 6. 系统深度优化设计 (在 115200 调试波特率下)

为了确保 6 个陀螺仪高频数据交互的绝对稳定，我们在方案中加入以下三项深度优化：

### 6.1 数据时间戳 (Data Timestamping)
由于多路传感器数据到达的时间存在微小抖动，我们在数据结构中引入高精度时间戳，以方便上位机或运动控制算法进行精确的“时空对齐”：
* **结构体修改**：在 `GyroData_t` 中新增 `uint32_t timestamp;` 变量。
* **时间戳捕获**：在 `HAL_UARTEx_RxEventCallback` 回调触发的**最开始**，调用 `HAL_GetTick()` 记录当前的毫秒数（或读取高精度定时器的 `TIMx->CNT`），确保时间的精准度。

### 6.2 调试串口非阻塞打印 (Non-blocking TX)
* **痛点**：波特率保持 `115200` 不变。如果采用阻塞式 `printf` 打印 6 个传感器的数据，CPU 耗时较长（约 30ms），这会造成严重的实时性下降和串口溢出。
* **优化策略**：
  * **DMA 异步发送**：开辟一个环形 TX 缓冲区。`printf` 的数据只管压入缓冲区，然后开启 `USART1` 的 DMA 异步发送（`HAL_UART_Transmit_DMA`）。发送过程不占用 CPU 运行时间。
  * **数据稀疏打印**：在调试阶段，主循环每隔 100ms 刷新打印一次数据，而不是每个传感器包都打印，防止 115200 串口带宽被彻底占满。

### 6.3 传感器上电自适应同步配置 (Auto-Configuration)
为了确保 6 个陀螺仪处于完全相同的通信参数与校准状态，在上电初始化时，单片机将自动通过各个串口向陀螺仪循环发送初始化配置：
* 自动设置回传速率（如 100Hz）。
* 自动发送加速度计校准指令。
* 自动保存配置参数至陀螺仪的 Flash。

---

## 7. 拓展至 6 个陀螺仪的步骤

如果后续需要从 2 个拓展至 6 个，只需三步：
1. 在 CubeMX 中打开对应的串口并开启其 DMA 接收配置。
2. 将 `GYRO_COUNT` 的宏定义修改为 6。
3. 在 `main()` 的初始化绑定部分，增加 `g_gyros[2].huart = &huart4;` 到 `g_gyros[5].huart = ...` 的绑定声明即可，核心解析和自愈代码完全不需要做任何改动。

---

## 8. 软件模拟串口 (Software UART) 读取第 6 个陀螺仪方案

为了保留 `USART1` 作为连接电脑的调试/数据打印通道（115200 波特率），我们在接满 6 个陀螺仪时，选择将**第 6 个陀螺仪通过“软件模拟串口”**的方式接入。

### 8.1 硬件引脚分配
* **模拟串口 RX**：选择 **`PB4`** 引脚（配置为普通输入，带上拉，开启下降沿外部中断 EXTI）。
* **模拟串口 TX**：选择 **`PB5`** 引脚（如果需要发送配置，配置为开漏输出；如果仅读取，可不接）。

### 8.2 模拟串口接收实现原理 (EXTI + 计数器定时采样)
9600 波特率下，传输一个比特（Bit）的时间约为：
$$T_{\text{bit}} = \frac{1}{9600\text{Hz}} \approx 104.16\mu\text{s}$$

利用 **外部中断 (EXTI4)** 与一个 **基本定时器 (如 TIM7)** 实现无阻塞接收：
1. **起始位检测**：
   * 将 `PB4` 配置为下降沿触发中断。当陀螺仪发送起始位（Start Bit，低电平）时，触发 `EXTI4` 中断。
   * 中断服务函数中：立刻关闭 `EXTI4` 外部中断，并配置并启动定时器 `TIM7`。
2. **定时采样（在 Bit 的正中间采样，最稳定）**：
   * 定时器 `TIM7` 配置为预分频 `83`（STM32F407 的 APB1 定时器时钟为 84MHz，分频后为 1MHz，即 **1计数 = 1微秒**）。
   * **第一次定时触发**：设为 $1.5$ 个 Bit 时间（即 $104 \times 1.5 = 156\mu\text{s}$）。此时刚好在第 1 个数据位的正中间，读取 `PB4` 电平存入缓冲。
   * **后续 7 次定时触发**：设为 $1.0$ 个 Bit 时间（$104\mu\text{s}$）。每次在数据位的正中间读取 `PB4` 并移位保存。
   * **第 9 次定时触发**：检测停止位（High），停止定时器 `TIM7`，将接收到的完整字节存入数据解析队列，重新开启 `EXTI4` 外部中断，等待下一包数据的起始位。
3. **数据接入解析流**：
   * 模拟串口接收到的字节放入缓冲数组后，同样定期调用 `ParseGyroBuffer` 函数进行标准校验解析，使其在软件架构上与硬件 DMA 串口完全无缝兼容。

这种方式的 CPU 占用率极低（9600波特率下每秒仅触发约 1000 次定时器中断），能完美实现第 6 个陀螺仪的高精度接收。


