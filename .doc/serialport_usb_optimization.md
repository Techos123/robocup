# USB 虚拟串口命令链路优化参考

## 目标

本方案用于整理 `Class_Serialport_USB` 与应用层之间的命令接收设计，目标是：

- 只有一个明确的 USB 接收入口。
- 变量名、处理函数和协议行为集中维护，避免整数索引与 `switch` 脱节。
- 一个 USB 数据包中的多条命令都能按顺序处理。
- 不使用与二维数组实际布局不匹配的 `char **`。
- 为后续从 USB 中断回调迁移到命令队列保留接口空间。

## 当前链路中的主要问题

当前代码中存在以下风险：

1. `serialport_usb.Init()` 使用默认参数，变量字典数量为 0，`p/i/d/v` 无法匹配。
2. `Class_Serialport_USB::Init()` 注册的是类内部静态回调，`main.c` 中负责修改 PID 的回调没有注册。
3. 解析器保存的是“最近一条命令”，一个 USB 包中如果包含多条命令，前面的命令会被覆盖。
4. 初始化接口使用 `char **` 表示固定宽度二维数组，类型表达不准确，也容易导致调用方强制转换。
5. 如果在 USB 中断中直接修改 PID 对象，后续增加复杂逻辑后可能拉长中断时间，或与主循环形成并发访问。

## 推荐的可读性方案: 命令描述表

### 1. 定义命令处理函数和命令表项

在 `dvc_serialport_usb.h` 中定义：

```cpp
typedef void (*Serialport_USB_Command_Handler)(float value);

struct Struct_Serialport_USB_Command
{
    const char *Name;
    Serialport_USB_Command_Handler Handler;
};
```

串口类的初始化接口改为接收命令表：

```cpp
void Init(
    const Struct_Serialport_USB_Command *Table,
    uint8_t Count);
```

类内部保存：

```cpp
const Struct_Serialport_USB_Command *Command_Table = nullptr;
uint8_t Command_Count = 0;
```

命令表应由静态或全局对象提供，确保 USB 接收过程中指针始终有效。

### 2. 在应用层定义处理函数

在 `main.c` 中定义独立、短小的处理函数：

```cpp
static void Apply_PID_KP(float value)
{
    motor_3508_0x201.PID_Omega.Set_K_P(value);
}

static void Apply_PID_KI(float value)
{
    motor_3508_0x201.PID_Omega.Set_K_I(value);
}

static void Apply_PID_KD(float value)
{
    motor_3508_0x201.PID_Omega.Set_K_D(value);
}

static void Apply_Target_Omega(float value)
{
    Target_Omega = value;
}
```

再集中定义命令表：

```cpp
static const Struct_Serialport_USB_Command
    Serialport_USB_Command_Table[] = {
        {"p", Apply_PID_KP},
        {"i", Apply_PID_KI},
        {"d", Apply_PID_KD},
        {"v", Apply_Target_Omega},
    };
```

初始化时传入表和表项数量：

```cpp
serialport_usb.Init(
    Serialport_USB_Command_Table,
    static_cast<uint8_t>(
        sizeof(Serialport_USB_Command_Table) /
        sizeof(Serialport_USB_Command_Table[0])));
```

这样新增命令只需要增加处理函数和一行表项，不需要维护 `case (0)`、`case (1)` 等隐式编号。

### 3. 让解析器直接查找命令表

`Find_Variable()` 可以改为返回命令表项指针：

```cpp
const Struct_Serialport_USB_Command *Find_Command(
    const uint8_t *Name,
    uint16_t Length) const;
```

查找时直接访问：

```cpp
const char *Candidate = Command_Table[i].Name;
```

不再需要以下成员和接口：

```cpp
char **Rx_Variable_List;
Get_Variable_Index();
Get_Variable_Value();
Variable_Index;
Variable_Value;
```

解析成功后立即调用对应处理函数：

```cpp
const Struct_Serialport_USB_Command *Command =
    Find_Command(Name, Name_Length);

if (Command != nullptr &&
    Parse_Variable_Value(Value_Text, Value_Length, &Value))
{
    if (Command->Handler != nullptr)
    {
        Command->Handler(Value);
    }
}
```

这种设计可以自然支持一个 USB 数据包包含多条命令，因为每遇到一条完整的行结束符，就会独立解析并分发一次。

## USB 接收回调的职责边界

接收链路应保持为：

```text
USB 中断
  -> USB_Hook_Receive()
  -> USB_ReceiveCallback()
  -> Serialport_USB_Static_Callback()
  -> Data_Process()
  -> Parse_Assignment()
  -> 命令处理函数或命令队列
```

`main.c` 不应再注册另一个原始 Buffer 回调，也不应再次调用 `USB_RxCpltCallback()`。否则同一数据可能被重复解析。

## 更稳妥的方案: 固定长度命令队列

上面的命令表方案最容易阅读，适合当前处理函数只做简单赋值的情况。但 USB 接收回调是在中断上下文中执行的。若后续命令需要日志、参数校验、Flash 写入或复杂计算，应改为：

1. USB 中断只完成字节组包、命令查找和入队。
2. 主循环从队列中取出命令。
3. 主循环调用 `Apply_PID_KP()` 等应用层处理函数。

命令结构可以设计为：

```cpp
struct Struct_Serialport_USB_Command_Event
{
    uint8_t Command_Id;
    float Value;
};
```

队列建议使用固定长度的单生产者、单消费者环形缓冲区。USB 中断是生产者，主循环是消费者。需要明确处理队列满的情况，例如丢弃最新命令并记录计数，或覆盖最旧命令。

## 参数校验建议

命令分发前应增加应用层范围检查：

```cpp
if (value < 0.0f || value > PID_KP_MAX)
{
    return;
}
```

尤其需要处理：

- `K_I == 0` 时 PID 积分限幅中的除零问题。
- PID 增益过大导致输出饱和。
- 目标转速超出电机和机构允许范围。
- 丢失电机反馈时禁止继续发送非零输出。

## 推荐的实施顺序

1. 先将 `char **` 变量表改为命令描述表。
2. 删除 `Variable_Index`、`Variable_Value` 这类“最近一次结果”状态。
3. 让 `Parse_Assignment()` 对每条完整命令立即分发。
4. 删除 `main.c` 中重复的原始 Buffer 回调链路。
5. 为 PID 参数和目标转速增加范围校验。
6. 确认 USB 回调只做轻量操作后，再决定是否加入环形命令队列。
7. 最后将控制周期从主循环延时改为硬件定时或基于时间戳调度。

## 维护检查清单

- [ ] USB 接收只注册一个上层入口。
- [ ] 命令表和命令处理函数在同一处可见。
- [ ] 新增命令不依赖隐式整数索引。
- [ ] 一个数据包内的多条命令按顺序处理。
- [ ] 命令表的生命周期覆盖整个运行期。
- [ ] 中断回调没有阻塞、打印或复杂计算。
- [ ] PID 参数有范围检查和零值保护。
- [ ] 电机掉线时输出被明确置零。
- [ ] 控制周期由真实时间保证，而不是依赖循环次数。
