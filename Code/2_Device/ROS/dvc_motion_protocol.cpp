/**
 * @file dvc_motion_protocol.cpp
 * @brief 基于USB CDC虚拟串口的X5运动控制二进制协议实现
 * @version 2.0
 * @date 2026-08-11
 */

/* 包含文件 ------------------------------------------------------------------*/

#include "dvc_motion_protocol.h"

#include <string.h>

/* 私有变量 ------------------------------------------------------------------*/

// 当前接管USB全局回调的协议对象，初始化前为空
Class_Motion_Protocol_USB *Class_Motion_Protocol_USB::Active_Instance = nullptr;

static_assert(sizeof(float) == 4U,
              "运动协议要求float必须为32位IEEE-754格式");

/* 函数实现 ------------------------------------------------------------------*/

/**
 * @brief 初始化协议解析状态并向底层USB驱动注册接收适配函数
 *
 * @param Callback_Function 每收到一条合法VEL、STOP或TURN帧时调用的回调
 *
 * 初始化会丢弃上一次遗留的半帧，并将三个统计计数器清零。USB_Init()只负责
 * 挂接现有USB CDC驱动，不会重新初始化USB外设，因此本函数必须在
 * MX_USB_DEVICE_Init()之后调用。
 */
void Class_Motion_Protocol_USB::Init(
    Motion_Protocol_Command_Callback Callback_Function)
{
  Command_Callback_Function = Callback_Function;
  Rx_Frame_Length = 0U;
  Statistics = {};

  Active_Instance = this;
  USB_Init(USB_Rx_Adapter);
}

/**
 * @brief 将USB驱动的全局接收回调转发给当前协议对象
 *
 * USB底层驱动只接受普通函数指针，不能直接注册非静态成员函数，因此使用该
 * 静态适配函数定位最后一次初始化的协议实例。尚未初始化时直接忽略数据。
 */
void Class_Motion_Protocol_USB::USB_Rx_Adapter(uint8_t *Rx_Data,
                                               uint16_t Length)
{
  if (Active_Instance != nullptr)
  {
    Active_Instance->USB_RxCpltCallback(Rx_Data, Length);
  }
}

/**
 * @brief 将本次USB字节流追加到协议缓冲区并解析全部完整帧
 *
 * 输入可以是半帧、一帧或多帧，也可以在合法帧前后带有噪声。每追加一个字节
 * 都会调用Process_Rx_Buffer()推进解析，因此无需假设USB数据包边界与协议帧
 * 边界一致。该函数不包含HAL_Delay或其他主动等待。
 */
void Class_Motion_Protocol_USB::USB_RxCpltCallback(uint8_t *Rx_Data,
                                                   uint16_t Length)
{
  if (Rx_Data == nullptr || Length == 0U)
  {
    return;
  }

  for (uint16_t i = 0U; i < Length; i++)
  {
    // 正常情况下，达到261字节的最大候选帧会立即被校验并移除。此处是边界
    // 保护：若内部状态意外异常，先丢弃头部一个字节，避免下一次写入越界。
    if (Rx_Frame_Length >= MOTION_PROTOCOL_MAX_FRAME_LENGTH)
    {
      Remove_Leading_Bytes(1U);
      Statistics.Format_Error_Count++;
    }

    Rx_Frame_Buffer[Rx_Frame_Length++] = Rx_Data[i];
    Process_Rx_Buffer();
  }
}

/**
 * @brief 从累计字节流中持续提取并处理完整协议帧
 *
 * 解析顺序如下：
 * 1. 搜索连续帧头AA 55，并丢弃帧头之前的噪声；
 * 2. 等待CMD和LEN字段到齐，根据LEN计算完整帧长度；
 * 3. 等待DATA和两个CRC字节全部到齐；
 * 4. 对CMD、LEN和DATA计算CRC-16/Modbus；
 * 5. CRC正确时严格检查命令、LEN和浮点数据，再分发给应用层；
 * 6. CRC错误时只丢弃当前帧头的第一个字节，然后重新搜索AA 55。
 *
 * 对于CRC正确但命令未知的帧，会依据LEN整体移除，不会将未知帧DATA中偶然
 * 出现的AA 55误认为新命令。
 */
void Class_Motion_Protocol_USB::Process_Rx_Buffer()
{
  while (true)
  {
    if (Rx_Frame_Length < 2U)
    {
      if (Rx_Frame_Length == 1U &&
          Rx_Frame_Buffer[0] != MOTION_PROTOCOL_FRAME_HEADER_0)
      {
        Rx_Frame_Length = 0U;
      }
      return;
    }

    uint16_t header_position = 0U;
    while (header_position + 1U < Rx_Frame_Length)
    {
      if (Rx_Frame_Buffer[header_position] == MOTION_PROTOCOL_FRAME_HEADER_0 &&
          Rx_Frame_Buffer[header_position + 1U] ==
              MOTION_PROTOCOL_FRAME_HEADER_1)
      {
        break;
      }
      header_position++;
    }

    if (header_position + 1U >= Rx_Frame_Length)
    {
      const bool keep_partial_header =
          Rx_Frame_Buffer[Rx_Frame_Length - 1U] ==
          MOTION_PROTOCOL_FRAME_HEADER_0;
      Rx_Frame_Length = keep_partial_header ? 1U : 0U;
      if (keep_partial_header)
      {
        Rx_Frame_Buffer[0] = MOTION_PROTOCOL_FRAME_HEADER_0;
      }
      return;
    }

    if (header_position > 0U)
    {
      Remove_Leading_Bytes(header_position);
    }

    if (Rx_Frame_Length < 4U)
    {
      return;
    }

    const uint8_t data_length = Rx_Frame_Buffer[3];
    const uint16_t frame_length =
        static_cast<uint16_t>(data_length) +
        MOTION_PROTOCOL_FRAME_OVERHEAD;

    if (Rx_Frame_Length < frame_length)
    {
      return;
    }

    const uint16_t calculated_crc =
        Calculate_CRC16_Modbus(Rx_Frame_Buffer + 2U,
                              static_cast<uint16_t>(data_length) + 2U);
    const uint16_t received_crc =
        static_cast<uint16_t>(Rx_Frame_Buffer[4U + data_length]) |
        (static_cast<uint16_t>(Rx_Frame_Buffer[5U + data_length]) << 8U);

    if (calculated_crc != received_crc)
    {
      Statistics.CRC_Error_Count++;
      // CRC错误时只丢弃当前AA，保留其余候选数据，以便找到内部或后续AA 55。
      Remove_Leading_Bytes(1U);
      continue;
    }

    if (Dispatch_Frame(Rx_Frame_Buffer[2],
                       Rx_Frame_Buffer + 4U,
                       data_length))
    {
      Statistics.Valid_Frame_Count++;
    }
    else
    {
      Statistics.Format_Error_Count++;
    }

    Remove_Leading_Bytes(frame_length);
  }
}

/**
 * @brief 校验命令专属数据格式并通知应用层
 *
 * VEL必须携带两个小端float32，共8字节；STOP的LEN必须为0；TURN必须携带
 * 一个小端float32，共4字节。所有浮点数都必须是有限值。TURN_DONE在下位机
 * 侧只允许发送，不作为有效输入命令接收。
 */
bool Class_Motion_Protocol_USB::Dispatch_Frame(uint8_t Command,
                                               const uint8_t *Data,
                                               uint8_t Data_Length)
{
  Struct_Motion_Protocol_Command motion_command = {};

  switch (Command)
  {
    case Motion_Protocol_Command_VEL:
    {
      if (Data_Length != 8U ||
          !Decode_Float32_LE(Data, &motion_command.Linear_X) ||
          !Decode_Float32_LE(Data + 4U, &motion_command.Angular_Z))
      {
        return false;
      }

      motion_command.Command = Motion_Protocol_Command_VEL;
      break;
    }
    case Motion_Protocol_Command_STOP:
    {
      if (Data_Length != 0U)
      {
        return false;
      }

      motion_command.Command = Motion_Protocol_Command_STOP;
      break;
    }
    case Motion_Protocol_Command_TURN:
    {
      if (Data_Length != 4U ||
          !Decode_Float32_LE(Data, &motion_command.Angle_Deg))
      {
        return false;
      }

      motion_command.Command = Motion_Protocol_Command_TURN;
      break;
    }
    default:
    {
      // TURN_DONE在运动下位机侧仅用于发送，其余命令字未在V2.0协议中定义。
      return false;
    }
  }

  if (Command_Callback_Function != nullptr)
  {
    Command_Callback_Function(motion_command);
  }

  return true;
}

/**
 * @brief 通过USB CDC发送固定的TURN_DONE回执帧
 *
 * @return uint8_t 返回USBD_OK、USBD_BUSY或USBD_FAIL
 *
 * USB发送为非阻塞方式。返回USBD_OK仅表示发送请求已成功提交，不表示主机
 * 已经完成读取；返回USBD_BUSY时调用方应稍后重试，不应在原地循环等待。
 * Tx_Turn_Done_Frame为对象成员，能够在USB异步发送期间持续保持有效。
 */
uint8_t Class_Motion_Protocol_USB::Send_Turn_Done()
{
  return USB_Transmit_Data(Tx_Turn_Done_Frame,
                           sizeof(Tx_Turn_Done_Frame));
}

/**
 * @brief 使用初始值0xFFFF计算CRC-16/Modbus
 *
 * 每个输入字节先与CRC低8位异或，再按最低位逐位右移；最低位为1时额外异或
 * 反射多项式0xA001。协议调用时Data指向CMD，Length等于2加LEN，不包含帧头。
 */
uint16_t Class_Motion_Protocol_USB::Calculate_CRC16_Modbus(
    const uint8_t *Data,
    uint16_t Length)
{
  if (Data == nullptr && Length > 0U)
  {
    return 0U;
  }

  uint16_t crc = 0xFFFFU;

  for (uint16_t i = 0U; i < Length; i++)
  {
    crc ^= Data[i];
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc = static_cast<uint16_t>((crc >> 1U) ^ 0xA001U);
      }
      else
      {
        crc = static_cast<uint16_t>(crc >> 1U);
      }
    }
  }

  return crc;
}

/**
 * @brief 解码一个小端IEEE-754 float32并拒绝NaN和正负无穷大
 *
 * 先按小端顺序将4个输入字节组合为uint32_t位模式，再检查指数位是否全为1。
 * 指数全为1表示NaN或无穷大，必须丢弃；合法位模式通过memcpy复制到float，
 * 避免未对齐地址访问和违反严格别名规则。
 */
bool Class_Motion_Protocol_USB::Decode_Float32_LE(const uint8_t *Data,
                                                  float *Value)
{
  if (Data == nullptr || Value == nullptr)
  {
    return false;
  }

  const uint32_t raw_value =
      static_cast<uint32_t>(Data[0]) |
      (static_cast<uint32_t>(Data[1]) << 8U) |
      (static_cast<uint32_t>(Data[2]) << 16U) |
      (static_cast<uint32_t>(Data[3]) << 24U);

  if ((raw_value & 0x7F800000U) == 0x7F800000U)
  {
    return false;
  }

  memcpy(Value, &raw_value, sizeof(raw_value));
  return true;
}

/**
 * @brief 从接收缓冲区头部移除指定数量的字节
 *
 * 若移除长度大于或等于当前有效长度，直接清空缓冲区；否则使用memmove将
 * 剩余字节前移。memmove允许源区域与目标区域重叠，适合此处的原地整理。
 */
void Class_Motion_Protocol_USB::Remove_Leading_Bytes(uint16_t Length)
{
  if (Length >= Rx_Frame_Length)
  {
    Rx_Frame_Length = 0U;
    return;
  }

  const uint16_t remaining_length =
      static_cast<uint16_t>(Rx_Frame_Length - Length);
  memmove(Rx_Frame_Buffer,
          Rx_Frame_Buffer + Length,
          remaining_length);
  Rx_Frame_Length = remaining_length;
}
