# 运动协议接口调用说明

| 项目 | 内容 |
| --- | --- |
| 适用协议 | `MOTION_PROTOCOL.md` V2.1 |
| 下位机协议类 | `Class_Motion_Protocol_USB` |
| 下位机头文件 | `Code/2_Device/ROS/dvc_motion_protocol.h` |
| 传输接口 | STM32 USB CDC虚拟串口 |
| 数据格式 | 二进制帧，不兼容旧ASCII文本协议 |

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

### 2.3 USB原始数据处理接口

```cpp
void USB_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);
```

正常使用时不需要在`main.c`中主动调用该函数。`Init()`已经向USB驱动注册内部适配回调，USB收到数据后会自动调用它。

该公有接口主要用于：

- 由其他接收入口主动转交原始字节流；
- 单元测试中注入半包、粘包或错误帧；
- 不调用`Init()`时单独测试协议解析器。

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

## 3. STM32下位机调用步骤

### 3.1 包含头文件并创建对象

```cpp
#include "dvc_motion_protocol.h"

Class_Motion_Protocol_USB motion_protocol;
```

### 3.2 定义应用层命令回调

以下示例直接调用底盘目标值接口，用于说明字段映射关系：

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
      // TURN是相对转角，不能直接传给需要绝对目标角度的Set_Target_Angle()。
      // 此处应将命令复制到主循环，由主循环完成Yaw解缠和目标角度计算。
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

### 3.3 初始化顺序

推荐顺序：

```cpp
MX_USB_DEVICE_Init();

car.Init();
car.Set_Chassis_Control_Type(Chassis_Control_Type_NORMAL);

motion_protocol.Init(Motion_Protocol_Command_Call_Back);
```

先初始化底盘，再注册运动协议回调，可以避免USB在底盘尚未初始化时收到命令。

当前工程的`main.c`仍使用旧的`Class_Serialport_USB`文本协议接口。仅将新协议源文件加入CMake不会自动替换旧调用，必须在后续接入时移除旧USB协议初始化，确保同一时刻只有一个上层模块调用`USB_Init()`。

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

概念上的正确关系为：

```text
绝对目标角度 = 收到TURN时的连续Yaw角度 + Command.Angle_Deg
```

项目IMU Yaw使用角度制，逆时针为正，并在`180 -> -180`处发生跳变。底盘完成TURN前必须实现连续Yaw解缠，或者在角度控制器中对误差进行周期归一化。否则跨越正负180度时可能得到错误的角度误差。

### 5.2 推荐TURN处理流程

```text
收到TURN
  -> 将目标线速度置零
  -> 保存当前连续Yaw
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
car.Set_Target_Angle(continuous_yaw_deg + Command.Angle_Deg);
```

其中`continuous_yaw_deg`必须是完成解缠后的连续角度，不能直接使用会在正负180度之间跳变的原始Yaw。

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

## 6. X5或ROS主机端发送方法

下位机协议类不提供`Send_Velocity()`或`Send_Turn()`，因为这些命令的方向是X5到下位机。X5上的ROS节点需要构造二进制帧并写入串口。

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

### 6.2 打开串口

```python
import serial


motion_serial = serial.Serial(
    port="/dev/ttyACM0",
    # USB CDC 没有波特率概念，此参数仅为串口库占位，实际不生效
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    xonxoff=False,
    rtscts=False,
    dsrdtr=False,
)
```

USB CDC设备在Linux上通常枚举为`/dev/ttyACM0`，个别驱动或发行版下可能是`/dev/ttyACM1`等；应以系统实际识别出的设备为准，或者通过udev规则创建固定别名。Windows下枚举为`COMx`。

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

### 6.4 发送相对转向命令

标准时序是先发送STOP，再发送一次TURN：

```python
motion_serial.write(make_stop_frame())
motion_serial.write(make_turn_frame(-90.0))
```

`-90.0`表示相对当前车身方向顺时针右转90度。发送TURN后，X5应进入`TURN_ACTIVE`状态，暂停发送新的VEL、STOP和TURN，直到收到合法`TURN_DONE`。

### 6.5 接收TURN_DONE

下位机返回：

```text
AA 55 83 00 60 80
```

Linux串口同样是字节流，主机端不能假设一次`read()`恰好返回完整6字节帧。主机端应按照帧头、LEN和CRC进行流式解析，只有收到合法的`CMD=0x83`、`LEN=0`帧后才能退出`TURN_ACTIVE`。

## 7. 已知接入状态

截至本文编写时：

- `Code/2_Device/ROS/dvc_motion_protocol.h/.cpp`已经完成协议封装；
- 新协议源文件已经加入`CMakeLists.txt`；
- `Core/Src/main.c`尚未初始化`Class_Motion_Protocol_USB`；
- `Core/Src/main.c`仍保留旧文本虚拟串口调用；
- TURN所需的连续Yaw、完成判定和稳定停止判定尚未接入。

因此，当前协议模块本身可以解析和封装数据，但在完成`main.c`及底盘控制接入前，X5发送的命令不会自动改变底盘运动目标。

## 8. 常见错误

| 错误 | 后果 | 正确做法 |
| --- | --- | --- |
| 同时初始化文本协议和运动协议 | 后初始化者覆盖USB接收回调 | 同一USB链路只使用V2.0运动协议 |
| 在回调中调用`HAL_Delay()` | USB接收缓冲区不能及时重新挂载 | 回调只复制数据并立即返回 |
| 将`Angle_Deg`直接传给`Set_Target_Angle()` | 相对角被误作绝对角 | 使用连续Yaw计算绝对目标角 |
| 在ANGLE模式下只设置`Target_Omega` | 角速度目标不参与当前控制 | VEL前切换到OMEGA模式 |
| `USBD_BUSY`时原地循环发送 | 阻塞主循环并破坏控制时序 | 在后续主循环中重试 |
| 收到TURN后继续发送VEL | 破坏转向互斥状态 | X5等待合法TURN_DONE |
| 使用ASCII字符串发送命令 | 下位机不会识别 | 发送带CRC的二进制帧 |
| 不校验主机收到的TURN_DONE | 噪声可能错误解除转向锁定 | 校验CMD、LEN和CRC |

