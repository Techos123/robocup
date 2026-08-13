# 运动协议接口调用说明

> 给第一次接触本协议的开发者：先阅读第 0 节，再根据你所在的一端选择第 3 节（STM32）或第 6 节（X5/Python）。

| 项目 | 内容 |
| --- | --- |
| 适用协议 | `MOTION_PROTOCOL.md` V2.0 |
| 下位机协议类 | `Class_Motion_Protocol_USB` |
| 下位机头文件 | `Code/2_Device/ROS/dvc_motion_protocol.h` |
| 传输接口 | STM32 USB CDC虚拟串口 |
| 数据格式 | 二进制帧，不兼容旧ASCII文本协议 |

## 0. 小白快速入门

### 0.1 这套接口解决什么问题

可以把它理解为“把一条运动命令装进固定格式的数据包，再通过 USB 发给下位机”。

| 你要做的事 | 应调用的接口 | 说明 |
| --- | --- | --- |
| STM32 接收 X5 的速度、停止、转向命令 | `Init()` + 命令回调 | `Init()` 只注册一次，收到合法帧后自动进入回调 |
| STM32 告诉 X5“转向完成” | `Send_Turn_Done()` | 只有实际完成转向并稳定停止后才调用 |
| 测试一段原始二进制数据 | `USB_RxCpltCallback()` | 可注入半包、粘包和错误帧，正常业务不必手动调用 |
| 查看通信是否出错 | `Get_Statistics()` | 只用于诊断，不要用它决定底盘运动 |
| X5 发送 VEL、STOP、TURN | Python 封帧函数 + `serial.write()` | 这些命令是 X5 发给 STM32 的，STM32 类没有 `Send_Velocity()` |

### 0.2 先记住通信方向

```text
X5/ROS  -------------------->  STM32 下位机
         VEL、STOP、TURN

X5/ROS  <--------------------  STM32 下位机
         TURN_DONE
```

因此：

- 在 STM32 端，`Init()` 是“开始接收”，不是“发送速度”。
- 在 STM32 端，速度和停止要在命令回调里交给 `car`。
- `TURN` 的角度是“相对当前方向”，而 `car.Set_Target_Angle()` 要的是“绝对方向”，两者不能直接相等。
- 这个协议是二进制协议，不要发送字符串 `VEL 0.3 0.5` 或 `STOP\n`。

### 0.3 一条最短调用链

```text
MX_USB_DEVICE_Init()
    -> motion_protocol.Init(命令回调)
    -> USB 收到字节
    -> 协议类自动拆包、验 CRC、验浮点数
    -> 命令回调(Command)
    -> 应用层设置 car 的目标值
```

回调只在“收到一条完整且合法的帧”时触发。半包不会触发，CRC 错误、长度错误和 NaN/Inf 也不会触发。

## 1. 模块职责

`Class_Motion_Protocol_USB`运行在运动控制下位机中，负责：

1. 接管现有USB驱动的接收回调。
2. 从USB连续字节流中解析V2.0二进制帧。
3. 校验帧头、LEN、CRC和浮点数合法性。
4. 将合法的`VEL`、`STOP`和`TURN`命令逐条交给应用层回调。
5. 在底盘确认转向完成后发送`TURN_DONE`回执。

该模块不负责：

1. 自动修改底盘目标值。
2. 对速度和转角执行机械安全限幅。
3. 将相对转角转换为底盘绝对目标角度。
4. 判断底盘是否已经完成转向并稳定停止。
5. 实现X5上的ROS节点或Linux串口发送程序。

通信方向如下：

| 命令 | 方向 | 下位机协议接口 |
| --- | --- | --- |
| `VEL 0x01` | X5 -> 下位机 | 解析后触发命令回调 |
| `STOP 0x02` | X5 -> 下位机 | 解析后触发命令回调 |
| `TURN 0x03` | X5 -> 下位机 | 解析后触发命令回调 |
| `TURN_DONE 0x83` | 下位机 -> X5 | 调用`Send_Turn_Done()`发送 |

## 2. 关键接口

### 2.1 初始化接口

```cpp
void Init(Motion_Protocol_Command_Callback Callback_Function);
```

作用：

- 清空协议接收状态和统计计数器。
- 保存应用层命令回调。
- 调用现有`USB_Init()`，将USB接收数据转交给协议解析器。

调用要求：

- 必须在`MX_USB_DEVICE_Init()`之后调用。
- 系统中只能初始化一个`Class_Motion_Protocol_USB`实例。
- 运动协议与`Code/2_Device/Virtual_Serialport`中的文本协议共用同一个USB接收回调，不能同时初始化。

最小调用示例：

```cpp
// 1. 先让 CubeMX 生成的 USB 外设完成初始化
MX_USB_DEVICE_Init();

// 2. 再让运动协议接管 USB 接收回调
motion_protocol.Init(Motion_Protocol_Command_Call_Back);
```

这里的 `Motion_Protocol_Command_Call_Back` 是你自己编写的函数名，不是系统自动提供的固定函数。函数名可以改成任何名字，但参数类型必须完全匹配第 2.2 节的回调类型。

不要这样写：

```cpp
motion_protocol.Init();                 // 错误：必须传入回调参数
motion_protocol.Init(Other_Callback);   // 只有在 Other_Callback 签名匹配时才正确
```

如果暂时只想测试解析器、不想让命令进入底盘，可以传 `nullptr`：

```cpp
motion_protocol.Init(nullptr);
```

此时帧仍会解析，统计计数仍会增加，但不会执行应用层命令。

### 2.2 命令回调类型

```cpp
typedef void (*Motion_Protocol_Command_Callback)(
    const Struct_Motion_Protocol_Command &Command);
```

回调参数：

```cpp
struct Struct_Motion_Protocol_Command
{
  Enum_Motion_Protocol_Command Command;
  float Linear_X;
  float Angular_Z;
  float Angle_Deg;
};
```

各字段含义：

| Command | 有效字段 | 单位 | 正方向 |
| --- | --- | --- | --- |
| `Motion_Protocol_Command_VEL` | `Linear_X` | `m/s` | 底盘x轴向前 |
| `Motion_Protocol_Command_VEL` | `Angular_Z` | `rad/s` | 绕z轴逆时针 |
| `Motion_Protocol_Command_STOP` | 无 | 无 | 无 |
| `Motion_Protocol_Command_TURN` | `Angle_Deg` | `deg` | 相对当前方向逆时针转动 |

回调在USB接收上下文中同步执行，必须快速返回。回调内禁止：

- 调用`HAL_Delay()`；
- 循环等待USB发送完成；
- 执行耗时计算；
- 保存`Command`引用供回调结束后使用。

如需在主循环中处理命令，应在回调中复制结构体内容，而不是保存引用。

最小可编译回调模板：

```cpp
void Motion_Protocol_Command_Call_Back(
    const Struct_Motion_Protocol_Command &Command)
{
  switch (Command.Command)
  {
    case Motion_Protocol_Command_VEL:
      // Command.Linear_X：m/s，前进为正
      // Command.Angular_Z：rad/s，逆时针为正
      break;

    case Motion_Protocol_Command_STOP:
      // STOP 没有有效的速度字段，直接把目标设为 0
      break;

    case Motion_Protocol_Command_TURN:
      // Command.Angle_Deg：相对当前方向，逆时针为正
      break;

    default:
      // 当前解析器不会把未知命令传到这里
      break;
  }
}
```

字段只在对应命令下有意义：

| 命令 | 应读取的字段 | 不要做的事 |
| --- | --- | --- |
| `VEL` | `Linear_X`、`Angular_Z` | 不要把 `Angular_Z` 当成角度制；它是 `rad/s` |
| `STOP` | 不需要读取字段 | 不要等待字段变成 0 来判断 STOP |
| `TURN` | `Angle_Deg` | 不要直接把相对角传给绝对角接口 |

回调参数是 `const ... &`，表示“只读引用”：

- 可以在回调里读取 `Command.Linear_X` 等字段；
- 不可以修改 `Command`；
- 回调返回后这份对象就失效；如果主循环稍后再处理，必须复制整个结构体或所需字段；
- 不要在回调中 `HAL_Delay()`，也不要循环等待队列或 USB 发送。

### 2.3 USB原始数据处理接口

```cpp
void USB_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);
```

正常使用时不需要在`main.c`中主动调用该函数。`Init()`已经向USB驱动注册内部适配回调，USB收到数据后会自动调用它。

该公有接口主要用于：

- 由其他接收入口主动转交原始字节流；
- 单元测试中注入半包、粘包或错误帧；
- 不调用`Init()`时单独测试协议解析器。

手动注入测试数据的示例：

```cpp
uint8_t test_frame[] = {
  0xAA, 0x55, 0x02, 0x00, 0x00, 0xD0  // STOP
};

motion_protocol.USB_RxCpltCallback(test_frame,
                                   sizeof(test_frame));
```

注意：`Rx_Data` 只是输入缓冲区，协议类会把需要跨 USB 数据包保留的字节复制到自己的内部缓冲区。调用者应保证传入指针在本次函数调用期间有效即可。`Length == 0` 或 `Rx_Data == nullptr` 时函数直接返回。

### 2.4 转向完成发送接口

```cpp
uint8_t Send_Turn_Done();
```

固定发送：

```text
AA 55 83 00 60 80
```

返回值：

| 返回值 | 含义 | 调用方处理 |
| --- | --- | --- |
| `USBD_OK` | 发送请求已提交 | 等待下一次业务事件 |
| `USBD_BUSY` | USB正在发送上一帧 | 稍后在主循环中重试 |
| `USBD_FAIL` | USB未枚举、未配置或参数异常 | 检查USB连接和初始化 |

`USBD_OK`只表示数据已经提交给USB协议栈，不表示X5已经读取完成。不得在原地循环调用，等待`USBD_BUSY`变为`USBD_OK`。

推荐写法：

```cpp
// 这个标志由你的应用层状态机维护
if (turn_completed && !turn_done_submitted)
{
  const uint8_t status = motion_protocol.Send_Turn_Done();

  if (status == USBD_OK)
  {
    // 只在提交成功后置 true，避免重复发送
    turn_done_submitted = true;
  }
  // USBD_BUSY：本次不置 true，下一次主循环再试
  // USBD_FAIL：记录错误并检查 USB 是否已枚举、是否完成初始化
}
```

下面这种“原地死循环等待”是错误的：

```cpp
while (motion_protocol.Send_Turn_Done() == USBD_BUSY)
{
  // 错误：会阻塞主循环，影响底盘控制时序
}
```

### 2.5 统计接口

```cpp
const Struct_Motion_Protocol_Statistics &Get_Statistics() const;
```

统计字段：

| 字段 | 含义 |
| --- | --- |
| `Valid_Frame_Count` | 命令、长度、CRC和浮点数据全部合法的帧数 |
| `CRC_Error_Count` | CRC错误的候选帧数 |
| `Format_Error_Count` | 命令未知、长度错误或浮点数非法的帧数 |

该接口只用于通信诊断，不应参与正常底盘控制逻辑。

读取示例：

```cpp
const Struct_Motion_Protocol_Statistics &statistics =
    motion_protocol.Get_Statistics();

// 这里只读，不要修改 statistics.Valid_Frame_Count
if (statistics.CRC_Error_Count > 0U)
{
  // 在调试输出中记录串口质量问题
}
```

统计值从 `Init()` 开始计数；再次调用 `Init()` 会清零。计数器是 `uint32_t`，长期运行溢出后会自然回绕。

### 2.6 CRC 计算接口

头文件还提供了一个静态工具函数：

```cpp
static uint16_t Calculate_CRC16_Modbus(const uint8_t *Data,
                                       uint16_t Length);
```

它只计算 CRC，不会自动加帧头，也不会自动发送数据。计算范围必须是 `CMD + LEN + DATA`，不包括 `AA 55`。

示例：

```cpp
const uint8_t body[] = {0x02, 0x00};  // CMD=STOP, LEN=0
const uint16_t crc =
    Class_Motion_Protocol_USB::Calculate_CRC16_Modbus(body,
                                                      sizeof(body));
// STOP 的 crc 应为 0xD000，发送顺序是 0x00、0xD0
```

调用规则：

- `Data == nullptr` 且 `Length != 0` 时返回 `0`，不要把这个结果当作有效校验；
- `Length` 是字节数，不是数组元素以外的其他单位；
- 发送时低字节先发，高字节后发。

## 3. STM32下位机调用步骤

### 3.1 包含头文件并创建对象

```cpp
#include "dvc_motion_protocol.h"

Class_Motion_Protocol_USB motion_protocol;
```

对象通常定义为全局对象，这样命令回调和主循环都能访问它。`motion_protocol` 只是变量名，也可以像当前工程一样命名为 `ros`，但同一份代码中的名字必须保持一致。

如果编译器提示找不到头文件，请确认：

- `CMakeLists.txt` 的 `target_include_directories()` 中包含 `Code/2_Device/ROS`；
- `target_sources()` 中包含 `Code/2_Device/ROS/dvc_motion_protocol.cpp`。

### 3.2 定义应用层命令回调

下面是最容易看懂的版本：收到 VEL 或 STOP 后直接设置底盘目标。它用于说明接口映射，暂不实现 TURN。

```cpp
void Motion_Protocol_Command_Call_Back(
    const Struct_Motion_Protocol_Command &Command)
{
  switch (Command.Command)
  {
    case Motion_Protocol_Command_VEL:
    {
      car.Set_Chassis_Yaw_Control_Type(Chassis_Yaw_Control_Type_OMEGA);
      car.Set_Target_Velocity_X(Command.Linear_X);
      car.Set_Target_Omega(Command.Angular_Z);
      break;
    }

    case Motion_Protocol_Command_STOP:
    {
      car.Set_Chassis_Yaw_Control_Type(Chassis_Yaw_Control_Type_OMEGA);
      car.Set_Target_Velocity_X(0.0f);
      car.Set_Target_Omega(0.0f);
      break;
    }

    case Motion_Protocol_Command_TURN:
    {
      // TURN 是相对转角，不能直接传给需要绝对角度的 Set_Target_Angle()。
      // 正式工程应把命令复制到队列，再由主循环处理。
      break;
    }

    default:
      break;
  }
}
```

该写法只适合展示接口关系。正式工程建议在回调中将命令复制到无阻塞队列，由主循环取出后调用`car.Set_*()`，原因如下：

- USB回调需要尽快返回，返回后底层才能重新挂载接收缓冲区；
- USB中断可能打断主循环中的底盘控制计算；
- 一个USB数据包可能包含多条协议帧，单个“最新命令”变量可能覆盖前一条命令；
- 标准TURN时序会连续出现STOP和TURN，队列能够保留完整顺序。

队列必须能在USB回调与主循环之间安全传递数据，并且禁止在回调中阻塞等待队列空间。

### 3.3 正式工程的非阻塞队列模板

下面是一个单生产者、单消费者的固定长度队列。USB 回调只负责复制命令，主循环负责改变底盘状态。队列满时丢弃新命令并计数，不会阻塞 USB 回调。

```cpp
// 队列实际最多保存 7 条命令，保留一个空位置用于区分“空”和“满”。
static constexpr uint8_t MOTION_COMMAND_QUEUE_SIZE = 8U;

Struct_Motion_Protocol_Command motion_command_queue[
    MOTION_COMMAND_QUEUE_SIZE] = {};
volatile uint8_t motion_command_write_index = 0U;
volatile uint8_t motion_command_read_index = 0U;
volatile uint32_t motion_command_drop_count = 0U;

void Motion_Protocol_Command_Call_Back(
    const Struct_Motion_Protocol_Command &Command)
{
  const uint8_t next_write_index = static_cast<uint8_t>(
      (motion_command_write_index + 1U) % MOTION_COMMAND_QUEUE_SIZE);

  if (next_write_index == motion_command_read_index)
  {
    // 队列已满：丢弃本条命令并立即返回，不能在这里等待。
    motion_command_drop_count++;
    return;
  }

  // 复制结构体内容，不能保存 Command 的引用或地址。
  motion_command_queue[motion_command_write_index] = Command;
  __DMB();  // 先写完结构体内容，再发布新的写下标。
  motion_command_write_index = next_write_index;
}

bool Get_Next_Motion_Command(Struct_Motion_Protocol_Command *Command)
{
  if (Command == nullptr)
  {
    return false;
  }

  // 主循环只在复制队列项的这几条指令期间屏蔽中断，避免 USB 回调打断复制。
  const uint32_t saved_primask = __get_PRIMASK();
  __disable_irq();

  if (motion_command_read_index == motion_command_write_index)
  {
    if (saved_primask == 0U)
    {
      __enable_irq();
    }
    return false;
  }

  *Command = motion_command_queue[motion_command_read_index];
  motion_command_read_index = static_cast<uint8_t>(
      (motion_command_read_index + 1U) % MOTION_COMMAND_QUEUE_SIZE);

  if (saved_primask == 0U)
  {
    __enable_irq();
  }
  return true;
}
```

在主循环中按顺序取出所有待处理命令：

```cpp
Struct_Motion_Protocol_Command command;

while (Get_Next_Motion_Command(&command))
{
  switch (command.Command)
  {
    case Motion_Protocol_Command_VEL:
      car.Set_Chassis_Yaw_Control_Type(Chassis_Yaw_Control_Type_OMEGA);
      car.Set_Target_Velocity_X(command.Linear_X);
      car.Set_Target_Omega(command.Angular_Z);
      break;

    case Motion_Protocol_Command_STOP:
      car.Set_Chassis_Yaw_Control_Type(Chassis_Yaw_Control_Type_OMEGA);
      car.Set_Target_Velocity_X(0.0f);
      car.Set_Target_Omega(0.0f);
      break;

    case Motion_Protocol_Command_TURN:
      // 在这里进入第 5 节介绍的 TURN 状态机。
      break;

    default:
      break;
  }
}
```

该模板假定 USB 回调是唯一生产者、主循环是唯一消费者。`__DMB()` 来自当前工程使用的 ARM CMSIS，用于保证命令内容先写完，再让主循环看到新的写下标。主循环复制一条命令时会短暂屏蔽中断，但临界区里只有一次小结构体复制和下标更新，不包含延时、循环或底盘计算。若以后使用 RTOS，优先改用 RTOS 提供的中断安全队列。

### 3.4 初始化顺序

推荐顺序：

```cpp
MX_USB_DEVICE_Init();

car.Init();
car.Set_Chassis_Control_Type(Chassis_Control_Type_NORMAL);

motion_protocol.Init(Motion_Protocol_Command_Call_Back);
```

先初始化底盘，再注册运动协议回调，可以避免USB在底盘尚未初始化时收到命令。

完整关系如下：

```cpp
// 全局区
#include "dvc_motion_protocol.h"

Class_Motion_Protocol_USB motion_protocol;

// main() 前定义回调，或先写同签名的函数声明
void Motion_Protocol_Command_Call_Back(
    const Struct_Motion_Protocol_Command &Command)
{
  // 推荐使用第 3.3 节的队列代码。
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  // 省略其他 CubeMX 外设初始化
  MX_USB_DEVICE_Init();

  // 先准备命令会使用到的底盘对象
  car.Init();
  car.Set_Chassis_Control_Type(Chassis_Control_Type_NORMAL);

  // 最后注册协议回调；Init() 内部已经调用 USB_Init()
  motion_protocol.Init(Motion_Protocol_Command_Call_Back);

  while (1)
  {
    // 在这里取出队列命令并执行底盘状态机。
  }
}
```

同一条 USB 接收链路不要再额外调用：

```cpp
USB_Init(USB_Callback);  // 使用 motion_protocol.Init() 时应避免重复注册
```

`motion_protocol.Init()` 内部已经调用 `USB_Init()` 并注册协议自己的适配回调。USB 驱动只有一个全局上层回调，后一次注册会覆盖前一次注册。

## 4. 将VEL和STOP交给底盘

### 4.1 VEL速度命令

收到`Motion_Protocol_Command_VEL`后，应先根据机械能力执行限幅，再设置底盘：

```cpp
const float target_velocity_x = Command.Linear_X;
const float target_omega = Command.Angular_Z;

car.Set_Chassis_Yaw_Control_Type(Chassis_Yaw_Control_Type_OMEGA);
car.Set_Target_Velocity_X(target_velocity_x);
car.Set_Target_Omega(target_omega);
```

必须切换到`Chassis_Yaw_Control_Type_OMEGA`。如果底盘仍处于`Chassis_Yaw_Control_Type_ANGLE`，`Set_Target_Omega()`写入的角速度目标不会用于当前航向控制输出。

协议层只拒绝NaN和无穷大，不提供机械安全范围。线速度和角速度上限必须由底盘应用层根据实际硬件确定。

### 4.2 STOP停止命令

收到`Motion_Protocol_Command_STOP`后：

```cpp
car.Set_Chassis_Yaw_Control_Type(Chassis_Yaw_Control_Type_OMEGA);
car.Set_Target_Velocity_X(0.0f);
car.Set_Target_Omega(0.0f);
```

STOP是软件目标置零，不等同于独立硬件急停。需要安全急停时必须使用独立硬件链路。

## 5. TURN相对定角转向

### 5.1 相对角度不能直接写入底盘

协议中的`Command.Angle_Deg`表示相对当前车身方向的转角，而现有：

```cpp
car.Set_Target_Angle(float Target_Angle);
```

保存的是绝对目标航向角。因此以下调用是错误的：

```cpp
car.Set_Target_Angle(Command.Angle_Deg);
```

对当前底盘常用的 `|Angle_Deg| <= 180°` 命令，概念上的关系为：

```text
绝对目标角度 = 收到 TURN 时的 Yaw 角度 + Command.Angle_Deg
```

项目 IMU Yaw 使用角度制，逆时针为正，并在 `180 -> -180` 处发生跳变。当前 `Class_Chassis_FW` 的角度环会把角度误差归一化到 `[-180°, 180°]`，所以普通的 `+90°` 或 `-90°` 转向可以跨越正负 180° 边界。

但是该“最短角误差”算法会主动选择最短旋转路径。例如请求 `+270°` 时，它可能按 `-90°` 执行。若业务需要一次旋转超过 180°，应用层必须实现连续 Yaw、累计转角或分段目标，不能直接套用当前角度环。协议本身没有限制角度范围，机械安全范围和大角度策略必须由应用层确定。

### 5.2 推荐TURN处理流程

```text
收到TURN
  -> 将目标线速度置零
  -> 保存当前有效Yaw
  -> 计算绝对目标角度
  -> 切换到底盘角度控制模式
  -> 设置绝对目标角度
  -> 持续检查角度误差和Yaw角速度
  -> 满足稳定停止条件并持续指定时间
  -> 调用Send_Turn_Done()
```

示意代码：

```cpp
car.Set_Target_Velocity_X(0.0f);
car.Set_Chassis_Yaw_Control_Type(Chassis_Yaw_Control_Type_ANGLE);
car.Set_Target_Angle(current_yaw_deg + Command.Angle_Deg);
```

其中 `current_yaw_deg` 是收到 TURN 那一刻的有效 IMU Yaw。目标只计算一次并保存，不能在每次主循环中都用“当前 Yaw + 相对角”重新计算，否则目标会随着车身一起移动，永远无法到达。

对于 `|Angle_Deg| <= 180°`，当前底盘的最短角误差归一化能够处理 Yaw 在正负 180° 的跳变。对于一次超过 180° 的指定方向旋转，则需要另外实现连续 Yaw 或累计转角状态机，并且不能继续使用当前“始终取最短路径”的完成误差作为唯一依据。

对当前底盘最常见的 `[-180°, 180°]` 相对转向，可以按以下顺序理解：

```cpp
// 示例变量：current_yaw_deg 是收到 TURN 当时的 IMU Yaw（单位 deg）。
const float target_yaw_deg = current_yaw_deg + Command.Angle_Deg;

// TURN 期间禁止平移。
car.Set_Target_Velocity_X(0.0f);

// 切换到角度环；只调用 Set_Target_Angle() 而不切模式是无效的。
car.Set_Chassis_Yaw_Control_Type(Chassis_Yaw_Control_Type_ANGLE);
car.Set_Target_Angle(target_yaw_deg);
```

这里的 `current_yaw_deg` 只是示例名称，不是协议类提供的变量。应用层应从已经判定为有效的 BMI088 姿态数据中读取它。若姿态无效，不要启动 TURN，也不要发送 `TURN_DONE`。

收到 TURN 后还需要保存这些应用层状态：

| 状态 | 用途 |
| --- | --- |
| `turn_active` | 表示正在转向，防止重复启动或提前回到速度模式 |
| `turn_target_yaw_deg` | 保存本次绝对目标角，不要每圈主循环重新累加相对角 |
| `turn_stable_timing` | 表示稳定计时是否已经开始 |
| `turn_stable_start_tick` | 记录本次连续进入稳定范围的起始时刻 |
| `turn_done_submitted` | 防止 `TURN_DONE` 成功提交后重复发送 |

计算跨正负 180° 的最短角度误差可参考：

```cpp
float Get_Shortest_Angle_Error_Deg(float target_deg,
                                   float current_deg)
{
  float error_deg = target_deg - current_deg;

  while (error_deg > 180.0f)
  {
    error_deg -= 360.0f;
  }
  while (error_deg < -180.0f)
  {
    error_deg += 360.0f;
  }

  return error_deg;
}
```

例如当前 Yaw 为 `170°`，左转 `30°` 的数学目标是 `200°`。IMU 到达该方向时可能显示为 `-160°`，代入上面函数后误差仍为 `0°`，不会误判成相差 `360°`。

### 5.3 发送TURN_DONE

只有同时满足以下条件后才能发送：

- 角度误差进入允许范围；
- Yaw角速度进入允许范围；
- 底盘目标线速度为0；
- 上述状态持续达到规定稳定时间。

示例：

```cpp
if (turn_completed && !turn_done_submitted)
{
  const uint8_t status = motion_protocol.Send_Turn_Done();

  if (status == USBD_OK)
  {
    turn_done_submitted = true;
  }
}
```

当返回`USBD_BUSY`时，保持`turn_done_submitted == false`，在后续主循环再次尝试。不要在当前循环中持续重试。

协议没有规定角度误差阈值、角速度阈值和稳定时间，这些参数必须根据底盘实际性能确定。

一个完整但仍需实车整定的判定框架如下。示例中的三个阈值不能直接视为本车安全参数：

```cpp
// 以下数值仅用于展示状态机写法，必须通过实车测试重新确定。
const float TURN_ANGLE_TOLERANCE_DEG = 2.0f;
const float TURN_YAW_RATE_TOLERANCE_RAD_S = 0.05f;
const uint32_t TURN_STABLE_TIME_MS = 200U;

// 每次收到一条新的 TURN 时，必须先执行：
// turn_stable_timing = false;
// turn_completed = false;
// turn_done_submitted = false;

const float yaw_error_deg = Get_Shortest_Angle_Error_Deg(
    turn_target_yaw_deg,
    current_yaw_deg);

const bool inside_stable_range =
    fabsf(yaw_error_deg) <= TURN_ANGLE_TOLERANCE_DEG &&
    fabsf(current_yaw_rate_rad_s) <= TURN_YAW_RATE_TOLERANCE_RAD_S;

if (!inside_stable_range)
{
  // 一旦离开稳定范围，本次连续稳定计时作废。
  turn_stable_timing = false;
}
else if (!turn_stable_timing)
{
  // 第一次进入稳定范围，从现在开始计时。
  turn_stable_start_tick = HAL_GetTick();
  turn_stable_timing = true;
}
else if ((uint32_t)(HAL_GetTick() - turn_stable_start_tick) >=
         TURN_STABLE_TIME_MS)
{
  turn_completed = true;
}

if (turn_completed && !turn_done_submitted)
{
  const uint8_t status = motion_protocol.Send_Turn_Done();
  if (status == USBD_OK)
  {
    turn_done_submitted = true;
    turn_active = false;
  }
}
```

使用 `fabsf()` 需要包含 `<math.h>`。`HAL_GetTick()` 的无符号减法写法可以处理计数器自然回绕。该状态机必须在主循环中反复运行，不能放在 USB 命令回调中等待 200 ms。

## 6. X5或ROS主机端发送方法

下位机协议类不提供`Send_Velocity()`或`Send_Turn()`，因为这些命令的方向是X5到下位机。X5上的ROS节点需要构造二进制帧并写入串口。

如果你只是想先验证串口是否通，建议先不接 ROS，使用下面的独立 Python 脚本发送 STOP。运行脚本前需要安装 `pyserial`：

```bash
python3 -m pip install pyserial
```

Windows 上可以把 `python3` 换成 `python`，串口名也要换成设备管理器中看到的实际端口，例如 `COM7`。

### 6.1 Python帧封装

```python
import math
import struct


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF

    for value in data:
        crc ^= value

        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1

    return crc & 0xFFFF


def make_frame(command: int, payload: bytes = b"") -> bytes:
    if not 0 <= command <= 0xFF:
        raise ValueError("command out of range")

    if len(payload) > 0xFF:
        raise ValueError("payload too long")

    body = bytes((command, len(payload))) + payload
    crc = crc16_modbus(body)

    return b"\xAA\x55" + body + struct.pack("<H", crc)


def make_velocity_frame(linear_x: float, angular_z: float) -> bytes:
    if not math.isfinite(linear_x) or not math.isfinite(angular_z):
        raise ValueError("velocity must be finite")

    return make_frame(0x01, struct.pack("<ff", linear_x, angular_z))


def make_stop_frame() -> bytes:
    return make_frame(0x02)


def make_turn_frame(angle_deg: float) -> bytes:
    if not math.isfinite(angle_deg):
        raise ValueError("angle must be finite")

    return make_frame(0x03, struct.pack("<f", angle_deg))
```

逐层理解这段代码：

1. `struct.pack("<f", value)` 把一个 Python 浮点数编码成 4 字节小端 `float32`。
2. `body` 只包含 `CMD + LEN + DATA`，CRC 只对这部分计算。
3. `struct.pack("<H", crc)` 把 16 位 CRC 按“小端”放入帧尾，因此低字节先发送。
4. `make_frame()` 最后才加上固定帧头 `AA 55`。

可以用下面几行检查示例帧是否与协议文档一致：

```python
print(make_stop_frame().hex(" ").upper())
# AA 55 02 00 00 D0

print(make_turn_frame(-90.0).hex(" ").upper())
# AA 55 03 04 00 00 B4 C2 07 79
```

### 6.2 打开串口

```python
import serial


motion_serial = serial.Serial(
    port="/dev/ttyUSB0",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    xonxoff=False,
    rtscts=False,
    dsrdtr=False,
    timeout=0.05,  # read() 最多等待 50 ms，避免接收循环永久阻塞
)
```

USB CDC设备在Linux上也可能枚举为`/dev/ttyACM0`。应使用系统实际识别出的设备路径，或者通过udev规则创建固定别名。

打开串口后可以先发送一条 STOP：

```python
motion_serial.write(make_stop_frame())
motion_serial.flush()
```

`write()` 返回后表示数据已经交给操作系统串口驱动；`flush()` 等待本地发送缓冲区写出。它不代表下位机已经执行完停止动作。

### 6.3 发送速度

```python
linear_x = 0.3
angular_z = 0.5

if abs(linear_x) < 0.0001 and abs(angular_z) < 0.0001:
    motion_serial.write(make_stop_frame())
else:
    motion_serial.write(make_velocity_frame(linear_x, angular_z))
```

相同速度帧的最小重复发送间隔为50ms，最高约20Hz。速度发生变化时可以立即发送新帧。

一个最小的周期发送示例：

```python
import time

try:
    while True:
        motion_serial.write(make_velocity_frame(0.3, 0.0))
        time.sleep(0.05)  # 相同速度帧不超过约 20 Hz
finally:
    motion_serial.write(make_stop_frame())
    motion_serial.close()
```

真实机器人测试时，建议先把速度改成 `0.0`，确认串口和 STOP 正常后再逐渐增加速度。

### 6.4 发送相对转向命令

标准时序是先发送STOP，再发送一次TURN：

```python
motion_serial.write(make_stop_frame())
motion_serial.write(make_turn_frame(-90.0))
```

`-90.0`表示相对当前车身方向顺时针右转90度。发送TURN后，X5应进入`TURN_ACTIVE`状态，暂停发送新的VEL、STOP和TURN，直到收到合法`TURN_DONE`。

推荐封装成一个带状态的函数，避免转向期间被普通 `/cmd_vel` 发送逻辑覆盖：

```python
turn_active = False


def request_turn(angle_deg: float) -> None:
    global turn_active

    if turn_active:
        return  # 当前转向未完成，丢弃新的 TURN 请求

    motion_serial.write(make_stop_frame())
    motion_serial.write(make_turn_frame(angle_deg))
    turn_active = True


def send_cmd_vel(linear_x: float, angular_z: float) -> None:
    if turn_active:
        return  # TURN_ACTIVE 期间不发送 VEL 或 STOP

    if abs(linear_x) < 0.0001 and abs(angular_z) < 0.0001:
        motion_serial.write(make_stop_frame())
    else:
        motion_serial.write(make_velocity_frame(linear_x, angular_z))
```

`request_turn(-90.0)` 表示右转 90 度；`request_turn(90.0)` 表示左转 90 度。角度是相对值，不是 IMU 的绝对 Yaw。

### 6.5 接收TURN_DONE

下位机返回：

```text
AA 55 83 00 60 80
```

Linux串口同样是字节流，主机端不能假设一次`read()`恰好返回完整6字节帧。主机端应按照帧头、LEN和CRC进行流式解析，只有收到合法的`CMD=0x83`、`LEN=0`帧后才能退出`TURN_ACTIVE`。

下面是一个只解析 `TURN_DONE` 的最小版本。它故意保留跨多次 `read()` 的缓存：

```python
rx_buffer = bytearray()


def poll_turn_done() -> bool:
    global turn_active

    rx_buffer.extend(motion_serial.read(motion_serial.in_waiting or 1))

    while True:
        header_index = rx_buffer.find(b"\xAA\x55")
        if header_index < 0:
            # 保留可能是半个帧头的最后一个 AA
            if rx_buffer[-1:] == b"\xAA":
                del rx_buffer[:-1]
            else:
                rx_buffer.clear()
            return False

        if header_index > 0:
            del rx_buffer[:header_index]

        if len(rx_buffer) < 4:
            return False

        data_length = rx_buffer[3]
        frame_length = data_length + 6
        if len(rx_buffer) < frame_length:
            return False

        frame = bytes(rx_buffer[:frame_length])
        body = frame[2:-2]
        received_crc = int.from_bytes(frame[-2:], "little")
        if crc16_modbus(body) != received_crc:
            del rx_buffer[0]
            continue

        del rx_buffer[:frame_length]
        if frame[2] == 0x83 and data_length == 0:
            turn_active = False
            return True
```

这个函数只负责“收到合法回执后解除锁定”，不负责判断 STM32 是否真的已经转到目标角度。`read()` 可以读到半帧、整帧或多帧，所以必须保留 `rx_buffer`。上面的串口配置将 `timeout` 设为 50 ms，因此没有数据时函数也会在有限时间内返回。

## 7. 已知接入状态

截至 2026-08-12，静态检查当前工程得到以下结果：

- `Code/2_Device/ROS/dvc_motion_protocol.h/.cpp`已经完成协议封装；
- 新协议源文件已经加入`CMakeLists.txt`；
- `Core/Src/main.c`已经创建 `Class_Motion_Protocol_USB ros`，并调用 `ros.Init(Motion_Protocol_Command_Call_Back)`；
- `main.c` 中的 `USB_Callback()` 会把原始数据交给 `ros.USB_RxCpltCallback()`；
- 初始化流程先调用 `USB_Init(USB_Callback)`，随后 `ros.Init()` 内部又调用一次 `USB_Init(USB_Rx_Adapter)`，后一次注册会覆盖前一次注册；虽然两条路径最终都进入同一个解析器，但第一次注册是冗余的，正式接入时应只保留 `ros.Init()` 这一条注册路径；
- 在当前构建范围内没有找到 `Motion_Protocol_Command_Call_Back` 的函数定义，因此当前代码会在编译阶段报“未声明”或后续链接错误；第 3.2、3.3 节给出了回调模板；
- `main.c` 已经向 `car.Set_Attitude_Feedback()` 提供 Yaw 和 Yaw 角速度，但没有看到 TURN 命令入队、相对目标角计算、稳定完成判定和 `Send_Turn_Done()` 调用；
- TURN 所需的应用层目标状态、完成判定和稳定停止判定尚未接入；若要支持一次超过 180° 的指定方向旋转，还需要连续 Yaw 或累计转角逻辑。

因此，协议类本身已经能够解析和发送协议帧，但当前 `main.c` 的应用层接入仍不完整。仅调用 `ros.Init()` 不会自动改变 `car` 的目标值；必须定义命令回调，并把命令安全地交给主循环处理。

本节是现状说明，不代表已经获得修改 `main.c` 的授权。实际接入源代码前，应先根据项目规则提交代码修改计划并获得确认。

## 8. 常见错误

### 8.1 快速排错表

| 现象或错误 | 可能后果/原因 | 正确做法 |
| --- | --- | --- |
| 找不到 `dvc_motion_protocol.h` | CMake 没有加入头文件目录 | 检查 `target_include_directories()` 是否包含 `Code/2_Device/ROS` |
| `Motion_Protocol_Command_Call_Back` 未声明 | 只写了 `ros.Init(...)`，没有定义回调 | 按第 3.2 或 3.3 节定义同签名函数，并确保定义或声明位于调用之前 |
| 回调始终不执行 | USB 未初始化、端口错误、帧格式或 CRC 错误 | 确认 `MX_USB_DEVICE_Init()` 在前、`Init()` 已调用，并查看 `Get_Statistics()` |
| 有效帧计数增加但底盘不动 | 回调没有把命令交给 `car`，或底盘控制仍为 `DISABLE` | 在主循环处理队列，并确认底盘已切换到 `Chassis_Control_Type_NORMAL` |
| 同时初始化多个上层 USB 协议 | 后一次 `USB_Init()` 覆盖前一次回调 | 同一 USB 链路只保留一个上层协议初始化入口 |
| 在回调中调用`HAL_Delay()` | USB接收缓冲区不能及时重新挂载 | 回调只复制数据并立即返回 |
| 将`Angle_Deg`直接传给`Set_Target_Angle()` | 相对角被误作绝对角 | 使用连续Yaw计算绝对目标角 |
| 在ANGLE模式下只设置`Target_Omega` | 角速度目标不参与当前控制 | VEL前切换到OMEGA模式 |
| `USBD_BUSY`时原地循环发送 | 阻塞主循环并破坏控制时序 | 在后续主循环中重试 |
| 收到TURN后继续发送VEL | 破坏转向互斥状态 | X5等待合法TURN_DONE |
| 使用ASCII字符串发送命令 | 下位机不会识别 | 发送带CRC的二进制帧 |
| 不校验主机收到的TURN_DONE | 噪声可能错误解除转向锁定 | 校验CMD、LEN和CRC |
| Python 报 `No module named serial` | 没有安装 `pyserial` | 运行 `python3 -m pip install pyserial` |
| Linux 打不开 `/dev/ttyUSB0` | 设备实际为 `/dev/ttyACM0` 或用户无串口权限 | 查看实际枚举路径并检查用户组/设备权限 |
| `CRC_Error_Count` 持续增加 | CRC 范围、字节序、串口数据或版本不一致 | CRC 只计算 `CMD + LEN + DATA`，并按低字节在前发送 |
| TURN 一直没有 `TURN_DONE` | 下位机没有完成判定，或 USB 返回 `BUSY/FAIL` | 检查姿态有效性、角度误差、Yaw 角速度、稳定计时和发送返回值 |

### 8.2 推荐调试顺序

1. 先让 Python 打印 STOP 帧，确认结果是 `AA 55 02 00 00 D0`。
2. 连接 STM32 后只发送 STOP，不要一开始就发送非零速度。
3. 查看 `Valid_Frame_Count` 是否增加；不增加时再看 `CRC_Error_Count` 和 `Format_Error_Count`。
4. 合法帧能收到后，确认命令回调确实入队，主循环确实出队。
5. 最后分别测试小速度 VEL 和 `+/-90°` TURN，并根据实车性能整定限幅及完成阈值。

### 8.3 三个统计计数怎么判断

| 统计现象 | 通常说明 |
| --- | --- |
| `Valid_Frame_Count` 增加 | 帧头、命令、长度、CRC 和浮点数都合法；还需继续检查应用层是否执行命令 |
| `CRC_Error_Count` 增加 | 找到了候选帧，但 CRC 不一致；优先检查 CRC 范围、大小端和传输字节 |
| `Format_Error_Count` 增加 | CRC 正确，但命令未知、LEN 不匹配，或浮点数为 NaN/Inf |
