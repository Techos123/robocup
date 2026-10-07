# `main.c` 详细解读

> 面向初学者的阅读笔记。本文按当前工作区中的 `Core/Src/main.c` 及其直接调用链整理，重点解释“程序什么时候做什么、数据从哪里来、最后怎样控制电机”。本文只做静态解读，不代表已经完成编译、下载或实机测试。

## 1. 先建立整体印象

`main.c` 是这个 STM32H723 机器人程序的总调度入口。它本身并不实现所有传感器、协议和 PID 算法，而是把各个模块接起来：

```text
上位机 X5
   │ USB CDC 二进制协议
   ▼
Class_Motion_Protocol_USB (ros)
   │ 合法命令回调
   ▼
ROS_Call_Back ──► V / Angle_Deg / Turn_Flag / Stop_Request
                                      │
                                      ▼
主循环 ──► Class_Chassis_FW (car) ──► 四个 DJI C620
                                      │
                                      ▼
                         CAN3 发送缓冲区 / FDCAN3

WHEELTEC AHRS ──UART7 + DMA空闲接收──► wt ──► 主循环读取Yaw
DJI C620 ───────FDCAN3接收中断──────► CAN3_Motor_Call_Back
TIM5 ───────────定时器更新中断──────► SYS_Timestamp
```

可以把它理解成两类工作同时存在：

1. **异步事件**：USB、UART、CAN 和 TIM5 由硬件中断触发，回调函数快速保存或处理新数据。
2. **主动轮询**：`main()` 中的无限循环反复读取最新姿态，执行底盘控制，并把最新电机指令发到 CAN。

项目没有使用 RTOS 任务；它采用“中断 + 裸机超级循环”的方式运行。

## 2. 为什么 `main.c` 里出现 C++ 语法

在根目录的 [CMakeLists.txt](../CMakeLists.txt#L43) 中，`main.c` 被显式标记为 `LANGUAGE CXX`，同时工程使用 C++17。因此：

- `Class_Motion_Protocol_USB ros;` 是 C++ 类对象；
- `car.Init()`、`wt.Get_Angle_Yaw()` 是成员函数调用；
- `const Struct_Motion_Protocol_Command &Command` 是 C++ 引用参数；
- 头文件虽然叫 `.h`，里面也可以声明 C++ 类。

文件名仍然叫 `main.c`，只是编译器把它按 C++ 翻译单元处理。这也是为什么 `main.c` 能直接使用 [crt_fw.cpp](../Code/3_Chariot/1_Module/Chassis/Four_whell/crt_fw.cpp) 等 C++ 模块。

## 3. 头文件：`main.c` 依赖了什么

### 3.1 CubeMX/HAL 生成的外设头文件

在 [main.c:20-27](../Core/Src/main.c#L20) 中：

| 头文件 | 在 `main.c` 中的作用 |
|---|---|
| `main.h` | STM32 HAL 总头文件、`Error_Handler` 声明和 GPIO 引脚定义 |
| `dma.h` | `MX_DMA_Init()` |
| `fdcan.h` | `hfdcan3` 和 `MX_FDCAN3_Init()` |
| `gpio.h` | `MX_GPIO_Init()` |
| `spi.h` | `MX_SPI2_Init()` |
| `tim.h` | `htim5`、`MX_TIM*_Init()`、定时器 HAL 类型 |
| `usart.h` | `huart7`、`MX_UART7_Init()`、`MX_USART10_UART_Init()` |
| `usb_device.h` | `MX_USB_DEVICE_Init()` |

这些头文件提供的是“外设句柄”和“初始化函数”。例如 `htim5` 不是定时器本身，而是 HAL 用来描述 TIM5 配置和状态的结构体对象。

### 3.2 项目业务模块

在 [main.c:31-40](../Core/Src/main.c#L31) 中：

- `bsp_power.h`：控制板载 24 V/5 V 电源输出。
- `crt_fw.h`：四轮底盘类 `Class_Chassis_FW`。
- `crt_music.h`：启动提示音。
- `drv_can.h`：CAN 初始化和周期发送函数。
- `drv_uart.h`：UART + DMA 接收初始化。
- `drv_usb.h`：USB CDC 接收/发送驱动。
- `dvc_ahrs_wheeltec.h`：WHEELTEC 姿态传感器解析和欧拉角接口。
- `dvc_motion_protocol.h`：USB 运动控制协议。
- `dvc_motor_dji.h`：DJI 电机数据解析、PID 和输出。
- `sys_timestamp.h`：基于 TIM5 的微秒时间戳。

被注释的 Serialplot 头文件不会参与编译，也不会注册串口绘图功能。

## 4. 类型和全局变量

### 4.1 转向状态枚举

[main.c:49-53](../Core/Src/main.c#L49) 定义了两个状态：

| 状态 | 数值 | 含义 |
|---|---:|---|
| `Turn_State_IDLE` | 0 | 没有正在执行一次离散转向 |
| `Turn_State_ACTIVE` | 1 | 已经锁定目标航向，正在闭环转向 |

`Turn_Flag` 是“收到 TURN、但还没有把相对角度转换成绝对目标角”的待处理标志；`Turn_State` 是“转向已经正式启动”的状态。二者分开可以避免在 USB 回调中直接读取姿态并进行复杂控制。

### 4.2 全局对象和共享数据

[main.c:71-83](../Core/Src/main.c#L71) 定义了全局对象：

| 变量 | 类型 | 初值/用途 |
|---|---|---|
| `ros` | `Class_Motion_Protocol_USB` | USB 运动协议对象，接收 VEL、STOP、TURN |
| `car` | `Class_Chassis_FW` | 四轮底盘控制对象，内部含 4 个 C620 电机 |
| `music` | `Class_Music` | 播放启动提示音 |
| `wt` | `Class_AHRS_WHEELTEC` | 姿态传感器对象 |
| `Yaw` | `float` | 主循环最近一次读取的航向角，单位度 |
| `Counter` | `uint32_t` | 主循环节拍计数，用于约 2 ms 和约 100 ms 任务 |
| `V` | `volatile float` | X 轴目标线速度，单位 m/s，前进为正 |
| `A` | `volatile float` | 当前代码没有使用的浮点共享变量 |
| `Angle_Deg` | `volatile float` | 最近一次 TURN 请求的相对转角，单位度，逆时针为正 |
| `Target_Angle` | `volatile float` | 当前绝对目标航向，单位度 |
| `Turn_Flag` | `volatile bool` | 收到并接受了 TURN，但尚未正式开始闭环 |
| `Stop_Request` | `volatile bool` | 收到 STOP，等待主循环执行一次停止处理 |
| `Turn_State` | `volatile Enum_Turn_State` | 当前离散转向状态 |

`volatile` 的意思是“每次都从内存重新读取，不能假设值不会被别的执行上下文改变”。它适合中断和主循环共享的变量，但**不等于互斥锁，也不保证复杂操作自动原子化**，所以代码又使用了 `__disable_irq()` 临界区。

`A` 目前没有在 `main.c` 中读写，属于预留变量或遗留变量。

## 5. `main.c` 中的回调和辅助函数

### 5.1 WHEELTEC UART 回调

[main.c:95-98](../Core/Src/main.c#L95) 的 `WT_UART_Call_Back()` 只是一个转发器：

```cpp
UART 驱动收到 UART7 DMA 空闲事件
        ↓
WT_UART_Call_Back(Buffer, Length)
        ↓
wt.UART_RxCpltCallback(Buffer, Length)
```

WHEELTEC 类负责查找帧头、校验 CRC、解码 AHRS 数据；`main.c` 不直接解析字节。UART 驱动使用 `UART_Init(&huart7, ..., 128)`，所以一次 DMA 接收缓冲区最大按 128 字节配置。

### 5.2 最短航向误差

[main.c:100-115](../Core/Src/main.c#L100) 计算：

```text
error = target - current
```

然后反复把误差压到 `[-180, 180]`：

- 大于 `180` 就减 `360`；
- 小于 `-180` 就加 `360`。

例如目标 `2°`、当前 `358°` 时，直接相减得到 `-356°`，归一化后得到 `4°`，表示顺时针/逆时针中较短的方向只差 4 度。这里的边界 `±180°` 是等价的，代码保留哪个符号并不影响后面的绝对值判定。

### 5.3 一次性取走 STOP 请求

[main.c:117-129](../Core/Src/main.c#L117) 的 `Main_Take_Stop_Request()` 是一个“取值并清零”的小工具：

1. 保存当前 `PRIMASK`，记住中断原来是否已关闭；
2. 关闭可屏蔽中断；
3. 复制 `Stop_Request`；
4. 立刻清零，防止同一个 STOP 被重复处理；
5. 如果进入函数前中断是开启的，再恢复开启状态。

这保证了 USB 回调不会在“读取”和“清零”之间插入，避免丢失或重复消费停止请求。

### 5.4 TIM5 更新回调

[main.c:131-137](../Core/Src/main.c#L131) 只处理 `TIM5`：调用 `SYS_Timestamp.TIM_3600s_PeriodElapsedCallback()`，让时间戳的溢出计数加一。

TIM5 在 [tim.c:86-90](../Core/Src/tim.c#L86) 中配置为预分频 `275-1`、自动重装 `3600000000-1`。项目时间戳模块要求预分频后为 1 MHz，因此计数器约每 1 微秒加一，约 3600 秒（1 小时）产生一次更新中断。这个 TIM5 中断**不是**主循环的 1 ms 控制节拍。

### 5.5 USB 运动命令回调

[main.c:139-177](../Core/Src/main.c#L139) 的 `ROS_Call_Back()` 接收已经由协议层完成帧头、长度、CRC 和浮点合法性校验的命令。协议命令定义见 [dvc_motion_protocol.h:41-69](../Code/2_Device/ROS/dvc_motion_protocol.h#L41)。

| 命令 | 条件 | `main.c` 的动作 |
|---|---|---|
| `VEL` | 当前没有待处理 TURN（`!Turn_Flag`） | 把 `Command.Linear_X` 保存到 `V`；协议仍保留 8 字节数据，但当前角速度 `Angular_Z` 不参与底盘控制 |
| `STOP` | 无额外条件 | `V=0`、清除转向标志、状态置为 IDLE、设置 `Stop_Request=true` |
| `TURN` | `Turn_State` 为 IDLE 且没有待处理 TURN | 保存相对角 `Angle_Deg`，把速度清零，设置 `Turn_Flag=true` |
| 其他 | - | 忽略 |

这个回调运行在 USB 接收上下文中。协议头文件明确要求回调不能延时或阻塞；当前回调只做赋值和状态切换，符合这个要求。

注意：`TURN` 的角度是“相对当前车身方向”的角度，范围由协议层限制为 `[-130°, 130°]`；主循环稍后才用 IMU 当前 Yaw 把它转换为绝对目标角。

### 5.6 CAN3 电机接收回调

[main.c:179-206](../Core/Src/main.c#L179) 按标准 CAN ID 把反馈分配给四个轮电机：

| CAN ID | `car.Motor_Wheel` 下标 |
|---:|---:|
| `0x201` | 0 |
| `0x202` | 1 |
| `0x203` | 2 |
| `0x204` | 3 |

FDCAN 驱动先把报文读入 `CAN3_Manage_Object.Rx_Buffer`，再调用这个回调。因此这里的 `Buffer` 参数当前没有直接使用，电机对象会从绑定的 CAN 管理对象中读取数据并完成编码器、转速、电流解析。

## 6. `main()` 启动流程

入口是 [main.c:214](../Core/Src/main.c#L214)。执行顺序很重要，因为很多模块必须先有 HAL 外设句柄，才能注册自己的回调。

### 6.1 CPU、HAL 和时钟

1. [main.c:224](../Core/Src/main.c#L224)：`SCB_EnableICache()` 打开指令缓存，提高从 Flash 取指令的效率。
2. [main.c:229](../Core/Src/main.c#L229)：`HAL_Init()` 复位 HAL 状态，初始化 Flash 接口和 SysTick，并准备中断优先级分组等基础设施。
3. [main.c:236](../Core/Src/main.c#L236)：`SystemClock_Config()` 使用 HSE，经 PLL1 生成系统时钟，再设置 HCLK、PCLK1/2/3/4 分频。
4. [main.c:239](../Core/Src/main.c#L239)：`PeriphCommonClock_Config()` 用 PLL2 提供 SPI2 和 FDCAN 的公共时钟。

PLL 的精确 MHz 数值还取决于板卡 HSE 输入频率；仅根据这段配置可以确认时钟来源和分频关系，不能在不知道 HSE 的情况下武断写出最终频率。

### 6.2 CubeMX 外设初始化

[main.c:246-256](../Core/Src/main.c#L246) 的顺序为：

```text
GPIO → DMA → UART7 → USB Device → FDCAN3
     → TIM5 → USART10 → TIM12 → SPI2 → TIM4 → TIM8
```

各项用途可以这样理解：

- **GPIO**：电源控制、SPI 片选、IMU 中断脚等。
- **DMA**：先准备 UART7 和其他外设的 DMA 通道。
- **UART7**：WHEELTEC 姿态传感器的串口。
- **USB Device**：建立 USB CDC 虚拟串口，后面 `ros.Init()` 才能接管它的接收回调。
- **FDCAN3**：四个 DJI C620 的反馈和控制总线。
- **TIM5**：微秒时间戳基准。
- **USART10**：本文件初始化了它，但当前 `main.c` 没有给它注册业务回调。
- **TIM12**：PWM 输出，供蜂鸣器等模块使用。
- **SPI2、TIM4、TIM8**：由 BMI088/相关底层模块使用，`main.c` 只负责完成 CubeMX 初始化。

若任意 HAL 初始化返回错误，通常会进入 [Error_Handler](#10-错误处理)，程序停在死循环中。

### 6.3 业务模块初始化

#### 电源、时间戳和 TIM5

```cpp
BSP_Power.Init(false, false, true);
SYS_Timestamp.Init(&htim5);
HAL_TIM_Base_Start_IT(&htim5);
```

对应 [main.c:258-260](../Core/Src/main.c#L258)：关闭两个 24 V 输出，仅打开 5 V 输出；绑定 TIM5 并开启更新中断。

#### UART7 和 WHEELTEC

```cpp
UART_Init(&huart7, WT_UART_Call_Back, 128);
wt.Init(&huart7);
wt.TIM_100ms_Alive_PeriodElapsedCallback();
```

对应 [main.c:262-265](../Core/Src/main.c#L262)。

- `UART_Init` 启动 UART7 的 DMA 空闲接收，并把收到的数据交给 `WT_UART_Call_Back`。
- `wt.Init` 等待至少一帧有效 AHRS 数据，最多等待约 3 秒；收到数据后再等待约 200 ms，并采集 100 个样本做零偏校准。这是一个**阻塞式启动步骤**，机器人必须保持静止。
- 因为 `wt.Init` 成功等待过数据，随后立即做一次 100 ms 存活检查，可以让第一轮主循环直接使用有效姿态，而不必再等满一个 100 ms 周期。

项目约定 `HAL_Delay(n)` 持续 `n+1 ms`，所以这里的 `HAL_Delay(200)` 实际约 201 ms；如果 IMU 一直没有响应，`HAL_Delay(10)` 的轮询节拍也约为 11 ms。

#### 底盘、电机和 CAN

```cpp
car.Init();
car.Set_Chassis_Yaw_Control_Type(Chassis_Yaw_Control_Type_ANGLE);
CAN_Init(&hfdcan3, CAN3_Motor_Call_Back);
```

对应 [main.c:267-270](../Core/Src/main.c#L267)：

- `car.Init()` 初始化四个 C620，绑定 ID `0x201` 到 `0x204`，默认使用轮端角速度控制；同时初始化航向角 PID 和四个轮速 PID。
- `Set_Chassis_Yaw_Control_Type(ANGLE)` 选择“航向角闭环”模式。之后底盘会用目标角和 IMU 当前 Yaw 算出目标角速度。
- `CAN_Init` 注册 `CAN3_Motor_Call_Back`、配置接收过滤器、打开接收通知并启动 FDCAN3。

`car.Init()` 先于 `CAN_Init()` 并不矛盾：电机对象先保存“将来使用 CAN3 管理对象”的指针，随后 `CAN_Init()` 再把 CAN3 句柄和回调真正绑定起来。

#### USB 协议、启动音乐和底盘使能

```cpp
ros.Init(ROS_Call_Back);
music.Play_Startup();
car.Set_Chassis_Control_Type(Chassis_Control_Type_NORMAL);
```

对应 [main.c:272-276](../Core/Src/main.c#L272)：

- `ros.Init` 把 USB CDC 接收回调 Hook 到运动协议解析器；协议层支持半包、粘包、噪声和 CRC 错误恢复。
- `music.Play_Startup` 播放提示音。按当前音符时长和项目延时约定，前台阻塞时间约为 `2+200+3+200+3+200+15+500=1123 ms`，期间主循环还没有进入控制逻辑，但中断仍可运行。
- 最后把底盘控制类型设置为 `NORMAL`。在这之前，底盘输出逻辑仍处于 `DISABLE` 状态。

## 7. 无限主循环：每一轮具体做什么

主循环位于 [main.c:282-381](../Core/Src/main.c#L282)。下面按实际执行顺序解释。

### 第 1 步：读取姿态

[main.c:284-292](../Core/Src/main.c#L284) 每轮先创建并清零一个 `Struct_Chassis_Attitude_Feedback`：

1. 如果 `wt.Get_Status()` 为 ENABLE，读取 `wt.Get_Angle_Yaw()` 和 `wt.Get_Omega_Yaw()`。
2. Yaw 写入 `Yaw` 和反馈结构，单位是度；Yaw 角速度写入 `Yaw_Rate_Rad_S`，单位是 rad/s。
3. 设置 `Valid=true`。
4. 如果 IMU 未被判定为在线，反馈保持零值且 `Valid=false`。

WHEELTEC 类对外的 Yaw 接口已经转换为角度制并归一化到 `[0, 360)`，符合本项目的 N100 实测约定；`main.c` 不再自行做弧度转角度。

### 第 2 步：把姿态交给底盘

[main.c:292](../Core/Src/main.c#L292) 调用 `car.Set_Attitude_Feedback(attitude_feedback)`。

底盘模块会：

- 拒绝无效、NaN 或无穷大的反馈；
- 保存当前 Yaw 和 Yaw 角速度；
- 在角度控制模式尚未有目标角时，把当前角初始化为保持目标；
- 后续在 2 ms 控制步骤中，用目标角和当前角计算航向 PID。

### 第 3 步：消费 STOP

[main.c:294-303](../Core/Src/main.c#L294) 如果 `Main_Take_Stop_Request()` 取到一个 STOP：

1. 把底盘目标 X 速度设为 0。
2. 如果本轮有有效姿态，把当前 Yaw 设为新的目标角，意思是“停下后保持此时朝向”。
3. 调用 `car.Reset_Wheel_Omega_PID()`，把四个轮速 PID 的积分/初始状态重置，减少停止后残余输出。

STOP 在 USB 回调里只置位，真正的底盘处理放在主循环，避免在 USB 中断里做较重的控制操作。

### 第 4 步：决定是否允许平动

[main.c:305-307](../Core/Src/main.c#L305) 只要有以下任一情况，就禁止 X 轴平动：

```text
Turn_State == ACTIVE 或 Turn_Flag == true
```

否则把 `V` 写入底盘目标 X 速度。这样 TURN 过程中即使上位机继续发送 VEL，车体也不会继续前进；只有转向结束回到 IDLE 后，新的 VEL 才能生效。

### 第 5 步：把待处理 TURN 转成绝对目标角

[main.c:309-332](../Core/Src/main.c#L309) 进入临界区后检查：

```text
当前状态是 IDLE
并且 Turn_Flag 为真
并且没有新的 Stop_Request
并且本轮姿态有效
```

满足时执行：

```text
Target_Angle = Angle_Deg + 当前 Yaw
```

其中 `Angle_Deg` 是相对车身方向的命令，当前 Yaw 是 IMU 的绝对方向。之后把目标角归一化到 `[-180, 180]`，写入 `car`，状态变成 `ACTIVE`，并清除 `Turn_Flag`。

之所以在临界区内完成检查、写目标和改状态，是为了防止 USB STOP/TURN 回调在中间插入，造成“已经开始转向但又被停止”之类的半更新状态。退出时按照进入前的 `PRIMASK` 恢复中断状态。

### 第 6 步：约每 2 ms 执行底盘控制

[main.c:334-338](../Core/Src/main.c#L334) 判断 `Counter % 2 == 0`，满足时调用：

```cpp
car.TIM_2ms_Resolution_PeriodElapsedCallback();
car.TIM_2ms_Control_PeriodElapsedCallback();
```

底盘内部的真实顺序见 [crt_fw.cpp:177-193](../Code/3_Chariot/1_Module/Chassis/Four_whell/crt_fw.cpp#L177)：

1. **自身解算**：由四个轮子的实际角速度计算车体 X 速度和角速度。
2. **航向控制**：在 ANGLE 模式下计算最短 Yaw 误差，运行航向 PID，得到目标角速度。
3. **运动学逆解**：把 X 速度和航向角速度换算成四个轮子的目标角速度。
4. **输出到电机**：运行四个 C620 的速度 PID，把结果写入 CAN3 的 `0x200` 发送缓冲区。

根据 [crt_fw.cpp:268-275](../Code/3_Chariot/1_Module/Chassis/Four_whell/crt_fw.cpp#L268)，轮端目标大致为：

```text
omega_x   = Target_Velocity_X / Wheel_Radius
omega_yaw = Resolved_Target_Omega * Wheel_Point_Y / Wheel_Radius

轮 0、1 = +omega_x - omega_yaw
轮 2、3 = -omega_x - omega_yaw
```

底盘坐标遵循项目约定：`x` 向前为正、`y` 向左为正、`z` 向上为正，逆时针角速度为正。轮子的正负号由底盘安装方向和源码中的轮号约定共同决定。

### 第 7 步：检查 TURN 是否完成

[main.c:340-364](../Core/Src/main.c#L340) 用当前反馈计算：

```text
abs(航向误差)       <= 2.0°
abs(当前航向角速度) <= 0.05 rad/s
abs(当前 X 速度)    <= 0.02 m/s
```

三个条件必须同时满足，而且 `Turn_State` 必须为 ACTIVE、姿态必须有效。

满足后，代码再次进入临界区并调用 `ros.Send_Turn_Done()`：

- 返回 `USBD_OK`：发送请求已经提交，状态改回 IDLE；
- 返回 `USBD_BUSY` 或 `USBD_FAIL`：保持 ACTIVE，下次循环继续尝试；
- 如果期间收到 STOP，则不发送完成回执。

协议层发送的是固定 6 字节帧 `AA 55 83 00 60 80`。USB 发送是非阻塞的，`USBD_OK` 只表示请求提交成功，不表示主机已经读完。

### 第 8 步：准备 CAN 输出

[main.c:365-366](../Core/Src/main.c#L365) 调用 `TIM_1ms_CAN_PeriodElapsedCallback()`。虽然名字带有“1 ms”，当前实现是在主循环里直接调用的，并非由一个 1 ms 硬件定时器触发。它最终发送：

```text
FDCAN3 标准帧 ID 0x200，8 字节，CAN3_0x200_Tx_Data
```

`car` 的电机控制函数先更新这块共享发送缓冲区，这一步再把缓冲区放入 FDCAN 发送 FIFO。由于主循环每轮只发送一帧，实际周期取决于主循环耗时和 CAN FIFO/总线状态。

### 第 9 步：约每 100 ms 做存活检查

[main.c:368-374](../Core/Src/main.c#L368) 先递增 `Counter`，达到 100 后清零并调用：

```cpp
wt.TIM_100ms_Alive_PeriodElapsedCallback();
car.TIM_100ms_Alive_PeriodElapsedCallback();
```

WHEELTEC 和四个 C620 都用“这一段时间内是否收到过新帧”的计数值判断在线状态：

- 收到过数据：状态 ENABLE；
- 没收到数据：状态 DISABLE，并清理相关 PID 积分或重新启动 UART 接收。

这里的“100 ms”是**名义周期**。因为它依赖 100 次主循环，每轮还包含控制计算、CAN 发送和中断处理，所以实际周期通常大于 100 ms，不能当作精确硬实时定时器。

### 第 10 步：延时后开始下一轮

[main.c:376](../Core/Src/main.c#L376) 调用 `HAL_Delay(0)`。按照本项目规则，它实际延时 **1 ms**，并不是“没有延时”。因此每轮循环至少被 SysTick 延迟约 1 ms，再加上函数执行时间和中断占用时间。

这带来三个重要结论：

1. `Counter % 2` 对应的是“约每两轮”，通常接近 2 ms，但不是严格的硬件 2 ms。
2. `Counter >= 100` 对应的是“约 100 轮”，实际通常超过 100 ms。
3. `TIM_1ms_CAN_PeriodElapsedCallback()` 的 1 ms 名称不能证明它真的以 1 ms 固定周期运行。

## 8. 三条核心业务流程

### 8.1 VEL：普通前后运动

```text
X5 发送 VEL
  ↓ USB CDC 接收
协议层校验 AA 55、LEN、CRC、两个 float
  ↓
ROS_Call_Back: V = Linear_X
  ↓ 主循环
没有转向时 car.Set_Target_Velocity_X(V)
  ↓ 约 2 ms 控制
底盘逆解算 → 四个轮速 PID → CAN 0x200
```

`Angular_Z` 当前只被协议解析器保存，没有在 `ROS_Call_Back` 中写入 `car.Set_Target_Omega()`，所以这份 `main.c` 不支持通过 VEL 直接控制角速度。

### 8.2 STOP：立即停止并保持当前方向

```text
X5 发送 STOP
  ↓
V=0，Turn_Flag=false，Turn_State=IDLE，Stop_Request=true
  ↓ 下一轮主循环
目标 X 速度=0
目标角=当前有效 Yaw
重置四个轮速 PID
```

STOP 的状态写入发生在 USB 回调，实际电机目标和 PID 重置发生在主循环。这样设计可让中断回调保持短小，但停止动作会等到主循环下一轮才执行。

### 8.3 TURN：相对转角闭环

```text
X5 发送 TURN(+/-角度)
  ↓
ROS_Call_Back 保存 Angle_Deg，设置 Turn_Flag
  ↓ 主循环读取当前 Yaw
Target_Angle = Angle_Deg + Yaw，并归一化
Turn_State = ACTIVE，平动速度强制为 0
  ↓ 约 2 ms 控制
航向角 PID → 目标角速度 → 四轮运动学逆解
  ↓
误差、角速度、X速度均进入阈值
  ↓
USB 发送 TURN_DONE，成功提交后回到 IDLE
```

转向期间收到 VEL 不会解除转向保护；收到 STOP 会清除转向状态，并阻止完成回执。

## 9. 中断、共享变量和时序注意事项

### 9.1 哪些代码可能在中断上下文运行

```text
UART7 DMA空闲中断
  → HAL_UARTEx_RxEventCallback
  → UART管理对象的回调
  → WT_UART_Call_Back

USB OTG HS中断
  → USB CDC Receive Hook
  → ROS协议解析
  → ROS_Call_Back

FDCAN3中断
  → HAL_FDCAN_RxFifo0Callback
  → CAN3_Motor_Call_Back
  → C620.CAN_RxCpltCallback

TIM5中断
  → HAL_TIM_PeriodElapsedCallback
  → 时间戳溢出计数
```

在这些上下文里不应该调用长时间阻塞函数。USB 协议回调的注释已经明确禁止延时；WHEELTEC 初始化中的延时只发生在主线程启动阶段，不是在接收回调中。

### 9.2 为什么要保存和恢复 `PRIMASK`

直接调用 `__enable_irq()` 可能破坏调用者原本的中断状态。例如某段代码本来就在更高层临界区中，函数返回后不应该擅自重新打开中断。这里使用：

```cpp
const uint32_t primask = __get_PRIMASK();
__disable_irq();
// 访问共享状态
if (primask == 0U) {
  __enable_irq();
}
```

这是一种“只恢复自己改变的状态”的写法。

### 9.3 主循环和中断之间的共享关系

| 共享数据 | 可能的写入方 | 可能的读取方 | 保护情况 |
|---|---|---|---|
| `V` | USB 回调 | 主循环 | `volatile`，没有包在临界区中 |
| `Angle_Deg` | USB TURN 回调 | 主循环 | `volatile`，在 TURN 启动检查时由临界区间接保护 |
| `Turn_Flag` | USB 回调、主循环 | USB 回调、主循环 | 关键状态切换有临界区 |
| `Stop_Request` | USB STOP 回调 | `Main_Take_Stop_Request`、TURN 完成检查 | 读取/清零和完成检查有临界区 |
| `Turn_State` | USB STOP 回调、主循环 | 主循环 | 关键状态切换有临界区 |
| 电机反馈 | FDCAN 中断 | 主循环 2 ms 解算 | 电机模块负责数据存储，主循环读取最新值 |

`volatile` 只能解决编译器缓存问题，不能替代设计层面的同步。当前代码对转向状态和停止请求处理得更谨慎；后续如果新增共享结构体或多字段状态，应继续使用短临界区或消息复制，避免在中断中直接做长计算。

### 9.4 启动阶段的阻塞

`wt.Init()` 和 `music.Play_Startup()` 都在主线程中等待/延时：

- `wt.Init()` 可能等待 IMU 首帧、融合稳定和 100 次校准采样；
- 启动音乐约阻塞 1.123 秒。

这并不等于全芯片完全停止：中断仍可接收数据。但在底盘控制类型被设为 `NORMAL` 前，主循环还没有开始主动输出控制。因此调试启动时应区分“中断在工作”和“主循环已经接管运动控制”。

## 10. 时钟、错误处理和断言

### 10.1 `SystemClock_Config`

[main.c:389-441](../Core/Src/main.c#L389) 完成三件事：

1. 配置电源供电和电压缩放；
2. 打开 HSE 并配置 PLL1；
3. 选择 PLL1 作为系统时钟，并设置 AHB/APB 分频。

任一 `HAL_RCC_*Config` 返回非 `HAL_OK` 都调用 `Error_Handler()`。

### 10.2 `PeriphCommonClock_Config`

[main.c:447-468](../Core/Src/main.c#L447) 配置 PLL2，并把 SPI123 和 FDCAN 的时钟源选为 PLL2。这样 SPI2 与 FDCAN3 使用独立于系统主时钟的公共时钟配置。

### 10.3 `Error_Handler`

[main.c:478-488](../Core/Src/main.c#L478) 先关闭中断，再永远停在 `while (1)`。这是典型的嵌入式“故障停机”策略：系统不会在外设初始化失败后继续运行，但也不会自动恢复。调试时如果程序卡死，应首先检查是否进入这里。

### 10.4 `assert_failed`

只有定义 `USE_FULL_ASSERT` 时才编译 [main.c:497-504](../Core/Src/main.c#L497)。当前函数体为空，不会输出文件名和行号；因此断言信息需要通过调试器或后续补充日志机制获取。

## 11. 初学者词汇表

| 术语 | 通俗解释 |
|---|---|
| HAL | ST 提供的硬件抽象库，用函数统一操作时钟、GPIO、UART、CAN、定时器等外设 |
| 外设句柄 | 一个结构体指针/对象，保存某个 UART、CAN 或 TIM 的配置和运行状态 |
| DMA | 让外设和内存直接搬运数据，CPU 不必逐字节参与 |
| 中断 | 硬件事件发生时暂时打断主循环，去执行对应处理函数 |
| 回调 | 把函数地址交给驱动，事件发生时由驱动反过来调用该函数 |
| CAN ID | CAN 报文的身份编号；这里 `0x201-0x204` 是四个电机反馈 ID，`0x200` 是发送组帧 ID |
| PID | 根据目标值和实际值的误差计算输出的闭环控制器 |
| 角速度 | 方向变化快慢，本文使用 rad/s；航向角使用度 |
| 临界区 | 暂时关闭中断的一小段代码，保证共享状态更新不会被打断 |
| `volatile` | 告诉编译器变量可能被外部执行上下文改变，不要把它当成永远不变的普通变量 |
| `static` 函数 | 只允许当前源文件使用的函数，避免与其他文件同名冲突 |

## 12. 建议的阅读顺序

如果想继续深入，而不是一次读完整个工程，可以按这个顺序：

1. 先看 [main.c:214-383](../Core/Src/main.c#L214)，掌握启动和主循环。
2. 再看 [crt_fw.cpp:177-315](../Code/3_Chariot/1_Module/Chassis/Four_whell/crt_fw.cpp#L177)，理解底盘速度、航向 PID、运动学逆解和电机输出。
3. 看 [dvc_motion_protocol.cpp:72-260](../Code/2_Device/ROS/dvc_motion_protocol.cpp#L72)，理解 USB 字节流如何变成 VEL/STOP/TURN。
4. 看 [dvc_ahrs_wheeltec.cpp:553-696](../Code/2_Device/AHRS/AHRS_WHEELTEC/dvc_ahrs_wheeltec.cpp#L553)，理解 UART 接收、校准和在线检测。
5. 看 [drv_can.cpp:163-241](../Code/1_Middleware/1_Driver/CAN/drv_can.cpp#L163)，理解 CAN 周期发送和接收中断。
6. 最后看 [dvc_motor_dji.cpp:798-940](../Code/2_Device/Motor/Motor_DJI/dvc_motor_dji.cpp#L798)，理解 C620 的反馈解析和速度 PID。

## 13. 一句话总结

`main.c` 的核心不是“完成所有算法”，而是建立时序和边界：初始化硬件，接收异步数据，把 USB 命令保存成安全的目标状态，在主循环中融合 IMU 与电机反馈，按约 2 ms 运行底盘闭环，按约 100 ms 检查设备存活，并把转向完成结果通过 USB 回传给上位机。
