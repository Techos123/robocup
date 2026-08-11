# BMI088 初始化与欧拉角 API 使用说明

## 文档目的

本文基于 `Code/2_Device/BSP/BMI088` 中的现有代码进行静态分析，说明：

- BMI088 驱动各文件的职责。
- 驱动正常工作所需的硬件、构建和算法依赖。
- 正确的初始化顺序。
- EXTI、SPI DMA 和周期任务之间的数据流及时序关系。
- 如何调用 API 获取 Yaw、Pitch、Roll。
- 当前代码中会阻止直接接入或影响结果可靠性的问题。

本文描述的是这套驱动的设计意图。当前仓库尚不具备直接编译、运行该驱动的条件，必须先完成“当前工程中的阻塞问题”一节列出的适配工作。

## 一、驱动结构

### 1. 顶层 BMI088 与姿态融合

文件：

- `Code/2_Device/BSP/BMI088/bsp_bmi088.h`
- `Code/2_Device/BSP/BMI088/bsp_bmi088.cpp`

主要职责：

- 管理加速度计和陀螺仪子对象。
- 响应两个传感器的数据就绪中断。
- 串行调度共享 SPI 总线上的 DMA 读取。
- 使用陀螺仪进行四元数状态预测。
- 使用归一化加速度进行 EKF 修正。
- 输出欧拉角、四元数、旋转矩阵、轴角和不同坐标系下的运动数据。

项目提供了一个全局对象：

```cpp
extern Class_BMI088 BSP_BMI088;
```

建议应用层统一使用该全局对象，不要再创建其他 BMI088 实例，否则现有回调路由、状态和应用层读取对象容易不一致。

### 2. 加速度计、温度和加热控制

文件：

- `Code/2_Device/BSP/BMI088/Accel/bsp_bmi088_accel.h`
- `Code/2_Device/BSP/BMI088/Accel/bsp_bmi088_accel.cpp`
- `Code/2_Device/BSP/BMI088/Accel/bsp_bmi088_accel_register.h`

默认配置：

| 项目 | 配置 |
|---|---|
| 加速度计量程 | `+/-24 g` |
| 输出数据率 | 1600 Hz |
| 滤波 | 代码注释标记为无滤波 |
| 数据就绪 | 映射到 INT1 |
| 加速度单位 | `m/s^2` |
| 本地重力加速度 | `9.7947 m/s^2` |
| 温度采样 | 由 128 ms 周期任务触发 |
| 加热目标 | 代码常量为 50 摄氏度 |

### 3. 陀螺仪

文件：

- `Code/2_Device/BSP/BMI088/Gyro/bsp_bmi088_gyro.h`
- `Code/2_Device/BSP/BMI088/Gyro/bsp_bmi088_gyro.cpp`
- `Code/2_Device/BSP/BMI088/Gyro/bsp_bmi088_gyro_register.h`

默认配置：

| 项目 | 配置 |
|---|---|
| 角速度量程 | `+/-2000 deg/s` |
| 输出数据率 | 2000 Hz |
| 带宽 | 532 Hz |
| 数据就绪 | 映射到陀螺仪中断输出 |
| 对外单位 | `rad/s` |

两个 `*_register.cpp` 文件目前没有实际逻辑；寄存器映射定义在对应的头文件中，并使用 `__attribute__((packed))` 保证成员偏移与芯片寄存器地址一致。

## 二、姿态解算的数据流

完整数据链路为：

```text
加速度/陀螺仪产生 Data Ready
        |
        v
GPIO EXTI 中断
        |
        v
BSP_BMI088.EXTI_Flag_Callback(GPIO_Pin)
        |
        | 记录 Ready_Flag 和微秒时间戳
        v
10 us 周期任务检查共享 SPI 是否空闲
        |
        v
启动对应传感器的 SPI DMA 读取
        |
        v
HAL SPI DMA 完成回调
        |
        v
BSP_BMI088.SPI_RxCpltCallback()
        |
        | 解析数据并设置 Update_Flag
        v
125 us 周期任务执行四元数 EKF
        |
        v
更新 Yaw/Pitch/Roll、四元数和旋转矩阵
        |
        v
BSP_BMI088.Get_Euler_Angle()
```

仅执行 `BSP_BMI088.Init()` 并不会持续产生欧拉角。EXTI、SPI DMA 完成回调、10 us 调度和 125 us 解算任务必须全部持续运行。

## 三、对外 API

### 1. 初始化

```cpp
void BSP_BMI088.Init();
```

顶层初始化函数依次执行：

1. 绑定 `SPI2_Manage_Object`。
2. 调用 `BMI088_Accel.Init(false)`。
3. 调用 `BMI088_Gyro.Init()`。
4. 将初始 Yaw 设置为 0。
5. 设置硬件初始化完成标志。

当前顶层接口硬编码关闭加热，应用层无法通过 `BSP_BMI088.Init()` 参数开启加热。

### 2. 获取欧拉角

```cpp
Class_Matrix_f32<3, 1> BSP_BMI088.Get_Euler_Angle() const;
```

返回值单位为弧度，排列顺序为：

| 索引 | 含义 | 绕轴 |
|---|---|---|
| `[0][0]` | Yaw | z 轴 |
| `[1][0]` | Pitch | y 轴 |
| `[2][0]` | Roll | x 轴 |

### 3. 其他常用 API

| API | 返回值 |
|---|---|
| `Get_Original_Accel()` | 校正后的加速度，单位 `m/s^2` |
| `Get_Original_Gyro()` | 校正后的角速度，单位 `rad/s` |
| `Get_Quaternion()` | 姿态四元数 |
| `Get_Rotation_Matrix()` | 姿态旋转矩阵 |
| `Get_Accel_Body()` | 去除重力后的机体坐标系加速度 |
| `Get_Gyro_Body()` | 机体坐标系角速度 |
| `Get_Accel_Odom()` | 里程计/大地坐标系加速度 |
| `Get_Gyro_Odom()` | 里程计/大地坐标系角速度 |
| `Get_Accel_Chi_Square_Loss()` | 加速度更新的卡方检验值 |
| `Get_Calculating_Time()` | 最近一次姿态计算耗时，代码实际单位为微秒 |

## 四、硬件和工程前置条件

### 1. SPI2

驱动硬编码使用 `SPI2_Manage_Object`，因此需要在 CubeMX 中提供 `hspi2`，并至少配置：

- SPI 主机模式。
- 全双工通信。
- 8 bit 数据宽度。
- 软件管理 NSS。
- SPI TX DMA 和 RX DMA。
- 对应 DMA 中断。

SPI 的 CPOL、CPHA、分频和最高通信频率没有在驱动中设置，必须根据 BMI088 数据手册和实际硬件配置确定。

### 2. GPIO

必须提供以下宏，并保证与电路图一致：

```cpp
BMI088_ACCEL__SPI_CS_GPIO_Port
BMI088_ACCEL__SPI_CS_Pin
BMI088_GYRO__SPI_CS_GPIO_Port
BMI088_GYRO__SPI_CS_Pin
BMI088_ACCEL__INTERRUPT_Pin
BMI088_GYRO__INTERRUPT_Pin
```

两个 CS 应配置为推挽输出，空闲状态为高电平。两个 Data Ready 引脚应配置为 EXTI，触发边沿需要与最终写入 BMI088 中断寄存器的有效电平一致。

### 3. 时间戳

BMI088 初始化中的 `Namespace_SYS_Timestamp::Delay_Millisecond()` 和运行中的数据时间戳都依赖 `SYS_Timestamp`。

因此必须在调用 BMI088 初始化之前执行：

```cpp
SYS_Timestamp.Init(&htim5);
HAL_TIM_Base_Start_IT(&htim5);
```

时间戳定时器应按 `sys_timestamp.h` 的要求以 1 MHz 计数，即每个计数为 1 us。

BMI088 使用的是自定义忙等待函数 `Delay_Millisecond(n)`，其目标延时为 `n ms`。这与项目规定的 `HAL_Delay(n)` 实际延时 `(n + 1) ms` 不是同一个函数。

### 4. 周期任务

| 周期 | 调用函数 | 作用 | 必要性 |
|---|---|---|---|
| 10 us | `TIM_10us_Calculate_PeriodElapsedCallback()` | 发起 SPI 读取并处理 20 us 传输超时 | 获取姿态必需 |
| 125 us | `TIM_125us_Calculate_PeriodElapsedCallback()` | EKF 预测、修正和姿态输出 | 获取姿态必需 |
| 128 ms | `TIM_128ms_Calculate_PeriodElapsedCallback()` | 温度读取调度和加热 PID | 加热/温度功能需要 |

10 us 和 125 us 不是整数倍关系，不适合用简单的整数软件分频从同一个 10 us 定时器生成；应使用独立定时器、统一的更高分辨率调度器，或者经过误差补偿的调度机制。

当前主循环中的 `HAL_Delay(0)` 实际延时 1 ms，而且主循环还包含其他操作，不能用于实现上述 10 us 或 125 us 实时周期。

### 5. 加热功能

如需启用加热，还必须具备：

- `htim3`。
- `TIM3 CH4 PWM`。
- `BSP_Power`。
- 电源电压 ADC 采集及其驱动。
- 50 摄氏度条件下得到的加速度和陀螺仪标定参数。

即使最终不启用加热，当前 `bsp_bmi088_accel.cpp` 仍直接引用 `htim3` 和 `BSP_Power`，所以现有实现仍需要解决这些符号和依赖关系。

## 五、推荐初始化顺序

完成当前工程兼容性修复后，推荐按以下顺序初始化：

1. `HAL_Init()`。
2. 配置系统时钟。
3. 初始化 GPIO。
4. 初始化 DMA。
5. 初始化 SPI2。
6. 初始化时间戳定时器及其他周期定时器。
7. 启动时间戳定时器。
8. 将两个 BMI088 CS 设置为高电平。
9. 向 SPI 驱动注册 BMI088 SPI 完成回调。
10. 如启用加热，先初始化电源电压采样和加热 PWM。
11. 调用 `BSP_BMI088.Init()`。
12. 启动 10 us、125 us 和可选的 128 ms 周期任务。

概念代码如下。该代码表示完成 SPI 驱动兼容和 CubeMX 配置后的目标接入方式，不能直接用于当前仓库：

```cpp
#include "bsp_bmi088.h"
#include "drv_spi.h"
#include "sys_timestamp.h"

static void BMI088_SPI2_Callback(
    uint8_t *tx_buffer,
    uint8_t *rx_buffer,
    uint16_t length)
{
    (void)tx_buffer;
    (void)rx_buffer;
    (void)length;
    BSP_BMI088.SPI_RxCpltCallback();
}

void App_Init()
{
    // MX_GPIO_Init、MX_DMA_Init、MX_SPI2_Init 和定时器初始化已完成。

    SYS_Timestamp.Init(&htim5);
    HAL_TIM_Base_Start_IT(&htim5);

    HAL_GPIO_WritePin(
        BMI088_ACCEL__SPI_CS_GPIO_Port,
        BMI088_ACCEL__SPI_CS_Pin,
        GPIO_PIN_SET);
    HAL_GPIO_WritePin(
        BMI088_GYRO__SPI_CS_GPIO_Port,
        BMI088_GYRO__SPI_CS_Pin,
        GPIO_PIN_SET);

    SPI_Init(&hspi2, BMI088_SPI2_Callback);
    BSP_BMI088.Init();

    // 在 BMI088.Init() 成功返回后启动姿态相关周期任务。
}
```

`BSP_BMI088.Init()` 内部使用异步 SPI DMA 读回寄存器，再通过 100 ms 忙等待给 DMA 回调留出执行时间。因此在调用它之前，SPI2、DMA、中断、时间戳和 SPI 回调注册必须已经可用。

## 六、回调接入

### 1. GPIO EXTI

在项目已有的 HAL EXTI 回调中转发引脚：

```cpp
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    BSP_BMI088.EXTI_Flag_Callback(GPIO_Pin);
}
```

该函数只记录数据就绪标志及其微秒时间戳，不直接进行 SPI 传输。

### 2. SPI DMA 完成

现有 `drv_spi.cpp` 已实现 `HAL_SPI_TxRxCpltCallback()`。正确做法是让 SPI 驱动在 SPI2 DMA 完成后调用注册的 `BMI088_SPI2_Callback()`，再由适配器调用：

```cpp
BSP_BMI088.SPI_RxCpltCallback();
```

不要在其他文件中重复定义第二个 `HAL_SPI_TxRxCpltCallback()`，否则会产生重复定义或破坏现有 SPI 统一分发机制。

### 3. 周期回调

应将以下调用合并到项目已有的定时器回调中，而不是重新定义一个同名 HAL 回调：

```cpp
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
    }
    else if (htim->Instance == BMI088_IO_TIMER_INSTANCE)
    {
        BSP_BMI088.TIM_10us_Calculate_PeriodElapsedCallback();
    }
    else if (htim->Instance == BMI088_EKF_TIMER_INSTANCE)
    {
        BSP_BMI088.TIM_125us_Calculate_PeriodElapsedCallback();
    }
    else if (htim->Instance == BMI088_HEATER_TIMER_INSTANCE)
    {
        BSP_BMI088.TIM_128ms_Calculate_PeriodElapsedCallback();
    }
}
```

上述 `BMI088_*_TIMER_INSTANCE` 是说明用占位符，必须替换为 CubeMX 中实际分配的定时器实例。

## 七、获取欧拉角

### 1. 基本调用

```cpp
Class_Matrix_f32<3, 1> euler = BSP_BMI088.Get_Euler_Angle();

float yaw_rad = euler[0][0];
float pitch_rad = euler[1][0];
float roll_rad = euler[2][0];
```

也可以使用 `auto`：

```cpp
auto euler = BSP_BMI088.Get_Euler_Angle();
```

### 2. 转换为角度

项目只提供了从角度到弧度的常量 `BASIC_MATH_DEG_TO_RAD`，因此可通过除法转换：

```cpp
float yaw_deg = yaw_rad / BASIC_MATH_DEG_TO_RAD;
float pitch_deg = pitch_rad / BASIC_MATH_DEG_TO_RAD;
float roll_deg = roll_rad / BASIC_MATH_DEG_TO_RAD;
```

### 3. 何时读取才有效

EKF 初始化需要至少一帧有效的加速度数据。第一次满足条件的 125 us 回调只完成 EKF 初始化；后续 125 us 回调才会计算并写入输出欧拉角。

因此不能在 `BSP_BMI088.Init()` 返回后立刻认为欧拉角有效。当前类没有公开 `Hardware_Init_Finished_Flag` 或 `EKF_Init_Finished_Flag`，也没有姿态有效性 API。正式使用前建议增加只读状态接口，例如：

```cpp
bool Get_Attitude_Valid_Flag() const;
```

不要仅依靠固定延时判断姿态是否有效，因为 SPI、传感器数据就绪和 EKF 首次初始化时间都可能变化。

### 4. 并发读取

欧拉角由 125 us 周期上下文写入，而应用主循环可能同时读取。返回一个三元素矩阵并不保证三个浮点数在并发场景下属于同一次更新。

对数据一致性要求较高时，应使用以下任一方式：

- 在很短的临界区内复制欧拉角快照。
- 使用双缓冲并在完整写入后原子切换索引。
- 在同一个姿态任务上下文中消费结果。

## 八、坐标系与欧拉角含义

本项目运动解算约定：

- x 轴向前为正。
- y 轴向左为正。
- z 轴向上为正。
- 绕 z 轴逆时针角速度为正。

BMI088 驱动将传感器的原始 x、y、z 数据直接送入 EKF，没有独立的板载安装方向映射。只有当 BMI088 传感器坐标轴与机器人坐标轴一致时，输出才能直接解释为上述机器人坐标系。

如果传感器实际安装方向不同，应在进入 EKF 前使用固定旋转矩阵或明确的轴交换、符号变换，将传感器数据转换到机器人坐标系。加速度和陀螺仪必须使用同一坐标变换。

初始姿态由归一化加速度计算：

```cpp
yaw   = 0.0f;
pitch = asinf(-accel_x_normalized);
roll  = atan2f(accel_y_normalized, accel_z_normalized);
```

这意味着上电时机器人应尽量保持静止，否则线加速度会被当成重力方向，导致初始 Pitch 和 Roll 偏差。

## 九、Yaw 的固有限制

该驱动只融合陀螺仪和加速度计，没有磁力计、视觉航向、轮式里程计航向或其他绝对 Yaw 观测。

因此：

- 初始 Yaw 被固定设置为 0。
- 加速度计只能观测重力方向，不能校正绕重力轴的 Yaw。
- Yaw 完全依赖陀螺仪积分，会随零偏和温漂逐渐漂移。
- `Get_Euler_Angle()[0][0]` 表示相对上电方向的航向，不是绝对地理航向。

如需长期稳定的 Yaw，必须增加额外观测源，并扩展 EKF 测量模型或在上层做航向融合。

## 十、EKF 行为

### 1. 首次初始化

收到第一帧有效加速度后，代码使用重力方向生成初始 Yaw-Pitch-Roll，再转换为归一化四元数，初始化四维状态 EKF。

### 2. 预测

陀螺仪数据作为 EKF 控制输入。代码使用数据就绪时间戳计算 `D_T`，并通过四元数微分方程执行预测。

当有效数据间隔超过 `0.1 s` 时，代码会清除 EKF 初始化完成标志，等待加速度重新初始化姿态。

### 3. 加速度修正

加速度先归一化，然后作为重力方向观测。代码计算卡方残差，只有残差不超过 `0.5` 时才执行 EKF 更新。

机器人快速平移、碰撞或振动时，加速度不再主要代表重力方向，卡方检验可以拒绝部分异常修正，但其阈值必须通过实际日志验证。

## 十一、时序与延时分析

### 1. 初始化耗时

加速度计和陀螺仪初始化包含大量固定的 100 ms 忙等待。按照当前循环中非零寄存器配置项估算，在所有 SPI 读写一次成功的情况下，初始化至少阻塞约 2.6 s。

如果修复“目标值为零时跳过写入”的问题，并对这些零值配置也执行一次写入和读回，最短耗时还会增加。

这些延时使用微秒时间戳忙等待，CPU 不会进入休眠，但中断仍可执行。初始化期间 SPI DMA 中断必须能够正常响应，否则寄存器影子值不会更新，初始化会永久停在 `while` 循环。

### 2. SPI 传输超时

顶层调度器将 SPI 传输超时设为 20 us，并每 10 us 检查一次。SPI 时钟、DMA启动开销和中断延迟必须保证正常事务能在 20 us 内完成，否则驱动会重复发起请求。

共享 SPI 同一时间只允许加速度、陀螺仪或温度中的一个事务进行。当前优先级为：

1. 加速度。
2. 陀螺仪。
3. 温度。

持续的加速度请求可能推迟陀螺仪或温度事务，因此需要在实机上记录实际等待时间。

### 3. EKF 计算周期

125 us 对应 8 kHz。传感器数据率只有 1600 Hz 和 2000 Hz，所以部分 EKF 周期没有新数据，但仍会根据当前状态和最新角速度生成姿态输出。

EKF 更新包含矩阵乘法和 3x3 矩阵求逆。必须测量 `Calculating_Time`，并保证最坏执行时间明显小于 125 us，同时检查它与 SPI DMA、EXTI、CAN、USB 等中断的优先级关系。

如果最坏执行时间接近或超过 125 us，应降低解算频率、优化矩阵实现，或让定时器中断只发布任务，由主循环/RTOS 高优先级任务完成 EKF，避免长时间占用中断上下文。

## 十二、当前工程中的阻塞问题

### 1. BMI088 未纳入构建

位置：`CMakeLists.txt:55-68`。

原因与影响：当前源文件列表没有 BMI088、SPI、Matrix、Quaternion、EKF、Power 等相关实现。当前工程即使能够正常构建，也不能说明 BMI088 代码可用。

建议：完成依赖版本统一后，将所需源文件和 include 路径显式加入目标；不要只添加 `bsp_bmi088.cpp`，否则会继续出现缺失类型或未定义符号。

### 2. 算法包含路径与仓库结构不一致

位置：

- `Code/2_Device/BSP/BMI088/bsp_bmi088.h:19-20`
- `Code/2_Device/BSP/BMI088/Accel/bsp_bmi088_accel.h:18-21`
- `Code/2_Device/BSP/BMI088/Gyro/bsp_bmi088_gyro.h:18-19`

原因与影响：BMI088 使用 `1_Middleware/Algorithm/...`、`1_Middleware/Driver/...`，当前仓库采用 `1_Middleware/2_Algorithm/...`、`1_Middleware/1_Driver/...`。此外当前仓库没有找到 Matrix、Quaternion 和 EKF 文件。

建议：找到与 BMI088 驱动同一版本的算法和驱动模块，统一目录结构及 include 路径。不要在缺少实现的情况下只修改字符串路径。

### 3. 现有 SPI 驱动与 BMI088 不兼容

位置：

- `Code/2_Device/BSP/BMI088/bsp_bmi088.cpp:55-81`
- `Code/2_Device/BSP/BMI088/Accel/bsp_bmi088_accel.cpp:217-262`
- `Code/2_Device/BSP/BMI088/Gyro/bsp_bmi088_gyro.cpp:129-174`
- `Code/1_Middleware/1_Driver/SPI/drv_spi.h:39-49`

原因与影响：BMI088 访问 `Activate_GPIOx`、`Activate_GPIO_Pin` 和 `Rx_Buffer_Length`，并调用 `SPI_Transmit_Receive_Data()`、`SPI_Transmit_Data()`；当前 SPI 驱动只有 `Now_GPIOx`、`Now_GPIO_Pin`、`Now_Rx_Length` 和 `SPI_Send_Receive_Data()`。代码无法直接编译。

当前 `drv_spi.h` 还包含 `stm32f4xx_hal.h`，而本工程目标是 STM32H7；`drv_spi.cpp` 的 include 路径也仍指向旧的 `User_Code` 目录。

建议：以 STM32H7 和当前 DMA 机制为基准重构一个统一 SPI 管理接口，明确片选有效电平、发送长度、接收长度和完成回调语义，再让 BMI088 适配该接口。

### 4. 当前 CubeMX 工程没有 BMI088 硬件配置

位置：

- `Core/Inc/stm32h7xx_hal_conf.h:75`
- `Core/Src/main.c:179-187`
- `Core/Inc/main.h:59-63`
- `Core/Src/gpio.c:42-52`

原因与影响：HAL SPI 模块未启用，没有 `MX_SPI2_Init()`、`hspi2`、SPI DMA 中断、BMI088 GPIO 宏或 EXTI 配置。`htim3` 也不存在。

建议：先根据硬件原理图在 CubeMX 中配置 SPI2、DMA、两个 CS、两个 Data Ready EXTI，以及需要的周期定时器；如保留加热，再配置 TIM3 CH4 PWM 和电源 ADC。

### 5. 加速度计软复位写错寄存器

位置：`Code/2_Device/BSP/BMI088/Accel/bsp_bmi088_accel.cpp:70-73`。

原因与影响：代码将软复位值 `0xb6` 写入 `ACC_PWR_CTRL_RW`，而寄存器映射中真正的软复位寄存器是 `ACC_SOFTRESET_WO`。芯片不会按预期软复位。

建议：将写入地址改为 `offsetof(Struct_BMI088_Accel_Register, ACC_SOFTRESET_WO)`，并在数据手册规定的复位等待时间后重新检查 Chip ID。

### 6. 目标寄存器值为零时不会执行写入

位置：

- `Code/2_Device/BSP/BMI088/Accel/bsp_bmi088_accel.cpp:83-95`
- `Code/2_Device/BSP/BMI088/Gyro/bsp_bmi088_gyro.cpp:64-76`

原因与影响：进入 `while` 前，代码先将本地影子寄存器设置为 `0x00`。当目标配置本身也是 `0x00` 时，循环条件立即为假，不会向芯片写入。

加速度计的 `ACC_PWR_CONF=0x00` 用于从默认挂起状态进入工作状态，该配置被跳过后，加速度计可能一直处于挂起状态，无法产生有效数据。

建议：每项配置至少无条件执行一次“写入、等待、读回”，随后再根据读回值决定是否重试；重试必须有次数和超时限制。

### 7. 初始化循环没有超时

位置：

- `Code/2_Device/BSP/BMI088/Accel/bsp_bmi088_accel.cpp:64-95`
- `Code/2_Device/BSP/BMI088/Gyro/bsp_bmi088_gyro.cpp:45-76`

原因与影响：Chip ID 和配置校验均使用无限 `while`。任何接线、供电、SPI模式、DMA、片选或回调错误都会让系统永久卡在启动阶段。

建议：让初始化函数返回明确状态；为 Chip ID、复位和每个寄存器配置设置最大重试次数、总超时和错误码。

### 8. 多寄存器写入长度可能少一个地址字节

位置：

- `Code/2_Device/BSP/BMI088/Accel/bsp_bmi088_accel.cpp:257-262`
- `Code/2_Device/BSP/BMI088/Gyro/bsp_bmi088_gyro.cpp:169-174`

原因与影响：发送缓冲区包含一个寄存器地址字节和 `Tx_Length` 个数据字节，但传给 SPI 的长度仅为 `Tx_Length`。调用该函数时可能少发送最后一个数据字节。

建议：根据最终统一后的 SPI API，将总发送长度明确设置为 `1 + Tx_Length`，并增加单字节、多字节写入测试。

### 9. 加热目标温度设置 API 没有生效

位置：

- `Code/2_Device/BSP/BMI088/Accel/bsp_bmi088_accel.h:210-213`
- `Code/2_Device/BSP/BMI088/Accel/bsp_bmi088_accel.cpp:197-200`

原因与影响：`Set_Target_Temperature()` 写入 `Target_Temperature`，但 PID 回调始终使用常量 `HEATER_TARGET_TEMPERATURE`，调用设置 API 不会改变目标温度。

建议：初始化 `Target_Temperature`，并让 PID 使用该成员；或者删除无效的 setter，只保留常量配置方式。

### 10. 启用加热时存在电压除零风险

位置：`Code/2_Device/BSP/BMI088/Accel/bsp_bmi088_accel.cpp:202-204`。

原因与影响：加热输出除以 `BSP_Power.Get_Power_Voltage()` 的平方。若 `BSP_Power` 未初始化、ADC 尚无有效数据或电压接近零，可能得到无穷大，并最终将 PWM 限制到最大值。

建议：启用加热前确认电源采样已经初始化；对电压增加有效性和最小阈值检查，异常时立即将 PWM 置零。

### 11. 顶层默认关闭加热，同时关闭温控标定补偿

位置：

- `Code/2_Device/BSP/BMI088/bsp_bmi088.cpp:40`
- `Code/2_Device/BSP/BMI088/bsp_bmi088.cpp:131-150`

原因与影响：顶层始终调用 `BMI088_Accel.Init(false)`。加速度仿射校正、加速度偏置和陀螺仪零偏只在 `Get_Heater_Enable()` 为真时应用，因此默认运行时不会使用这些 50 摄氏度标定参数。

建议：明确区分“常温标定”和“恒温标定”；让顶层 `Init()` 接收加热配置，并为不同温度策略使用匹配的标定参数。

### 12. 缺少姿态有效状态 API

位置：`Code/2_Device/BSP/BMI088/bsp_bmi088.h:124-127`。

原因与影响：硬件和 EKF 初始化标志都是 protected，应用层只能读取欧拉角，无法判断它是默认值、旧值还是有效融合结果。

建议：增加只读的硬件状态、EKF状态或统一姿态有效标志；发生 SPI 超时、数据超时或 EKF 重置时同步更新该状态。

### 13. `Get_Calculating_Time()` 单位注释错误

位置：

- `Code/2_Device/BSP/BMI088/bsp_bmi088.cpp:339`
- `Code/2_Device/BSP/BMI088/bsp_bmi088.h:324-331`

原因与影响：实现返回两个微秒时间戳之差，实际单位为微秒；头文件注释写成秒，容易造成实时性判断错误。

建议：将注释改为微秒，或将 API 名称明确为 `Get_Calculating_Time_Us()`。

### 14. 陀螺仪头文件存在未实现 API

位置：`Code/2_Device/BSP/BMI088/Gyro/bsp_bmi088_gyro.h:52`。

原因与影响：头文件声明了 `Get_Callibrated_Gyro()`，但没有找到对应定义。应用层调用后会产生编译或链接问题；名称中的 `Callibrated` 也存在拼写错误。

建议：实现并统一为 `Get_Calibrated_Gyro()`，或者删除未使用声明，避免提供不可用接口。

## 十三、推荐联调顺序

为缩小故障范围，建议按以下顺序逐步验证：

1. 仅配置 SPI2、DMA 和两个 CS，不启用 EKF及加热。
2. 分别读取加速度计 Chip ID `0x1e` 和陀螺仪 Chip ID `0x0f`。
3. 修复软复位和零值配置问题，验证全部配置寄存器读回值。
4. 轮询读取六字节加速度和六字节角速度，确认单位及轴方向。
5. 接入两个 Data Ready EXTI，记录实际中断频率。
6. 接入共享 SPI 的 10 us 调度，验证没有并发 DMA或重复片选。
7. 静止放置机器人，检查加速度模长接近 `9.7947 m/s^2`、角速度接近零。
8. 接入 125 us EKF，检查初始 Pitch/Roll 和四元数模长。
9. 分别绕机器人 x、y、z 正方向旋转，核对 Roll、Pitch、Yaw 的符号。
10. 记录 Yaw 静止漂移、卡方残差和最坏计算耗时。
11. 最后再启用温度读取、PWM 加热和恒温标定补偿。

## 十四、最小使用示例

完成所有前置适配并确认姿态有效后，应用层读取代码可以保持很简单：

```cpp
void Read_BMI088_Attitude()
{
    const auto euler = BSP_BMI088.Get_Euler_Angle();

    const float yaw_rad = euler[0][0];
    const float pitch_rad = euler[1][0];
    const float roll_rad = euler[2][0];

    const float yaw_deg = yaw_rad / BASIC_MATH_DEG_TO_RAD;
    const float pitch_deg = pitch_rad / BASIC_MATH_DEG_TO_RAD;
    const float roll_deg = roll_rad / BASIC_MATH_DEG_TO_RAD;

    // 在此处使用或发送姿态数据。
}
```

实际应用应在姿态有效标志为真时读取，并根据并发模型保证三个角来自同一份完整快照。

## 十五、接入检查表

- [ ] BMI088 所需 Matrix、Quaternion、EKF 版本已补齐。
- [ ] SPI 驱动接口与 BMI088 完全一致。
- [ ] STM32H7 HAL SPI 已启用。
- [ ] SPI2、TX/RX DMA 和 DMA IRQ 已配置。
- [ ] 两个 CS 初始为高电平。
- [ ] 两个 Data Ready EXTI 已配置且极性正确。
- [ ] 时间戳在 BMI088 初始化前已经运行。
- [ ] 加速度计软复位地址已修复。
- [ ] 零值寄存器配置会被实际写入。
- [ ] 初始化具有超时和错误返回。
- [ ] 10 us 数据调度稳定运行。
- [ ] 125 us EKF 的最坏执行时间满足约束。
- [ ] 传感器轴已转换到 x 前、y 左、z 上机器人坐标系。
- [ ] 应用层能够判断姿态是否有效。
- [ ] 已接受 Yaw 漂移，或已经增加绝对航向观测。
- [ ] 如启用加热，电源 ADC、PWM 和安全保护均已验证。，，

