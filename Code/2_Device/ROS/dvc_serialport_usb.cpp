/**
 * @file Serialport_USB.cpp
 * @author Techos (3069104675@qq.com)
 * @brief USB CDC虚拟串口下的JustFloat发送与文本变量赋值接收封装
 * @version 0.2
 *
 * @date 2026-8-7  完成USB CDC虚拟串口下的JustFloat发送与文本变量赋值接收封装
 * @copyright SUST-RoboZB 2026
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_serialport_usb.h"

#include <float.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

void Class_Serialport_USB::Init(USB_Callback Callback_Function,
                                uint8_t __Rx_Variable_Assignment_Num,
                                char **__Rx_Variable_Assignment_List)
{
  Rx_Variable_Num = (__Rx_Variable_Assignment_List != nullptr)
                        ? __Rx_Variable_Assignment_Num
                        : 0U;
  Rx_Variable_List = __Rx_Variable_Assignment_List;

  Rx_Line_Length = 0U;
  Rx_Discard_Until_Line_End = false;
  Variable_Index = -1;
  Variable_Value = 0.0f;

  USB_Init(Callback_Function);
}

void Class_Serialport_USB::USB_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
  if (Rx_Data == nullptr || Length == 0U)
  {
    return;
  }

  Data_Process(Rx_Data, Length);
}

uint8_t Class_Serialport_USB::JustFloat_Output()
{
  if (Data_Number == 0U)
  {
    return USBD_FAIL;
  }

  const uint16_t payload_length =
      static_cast<uint16_t>(Data_Number * sizeof(float));
  const uint16_t frame_length =
      static_cast<uint16_t>(payload_length + 4U);

  if (frame_length > USB_BUFFER_SIZE)
  {
    return USBD_FAIL;
  }

  uint8_t *tx_buffer = Tx_Buffer[Tx_Buffer_Index];

  for (uint8_t i = 0U; i < Data_Number; i++)
  {
    float channel_value = Data_Value[i];
    if (Data_Reader[i] != nullptr)
    {
      if (Data_Address[i] == nullptr)
      {
        return USBD_FAIL;
      }
      channel_value = Data_Reader[i](Data_Address[i]);
    }

    memcpy(tx_buffer + i * sizeof(float),
           &channel_value,
           sizeof(channel_value));
  }

  tx_buffer[payload_length + 0U] = 0x00U;
  tx_buffer[payload_length + 1U] = 0x00U;
  tx_buffer[payload_length + 2U] = 0x80U;
  tx_buffer[payload_length + 3U] = 0x7fU;

  const uint8_t status = USB_Transmit_Data(tx_buffer, frame_length);
  if (status == USBD_OK)
  {
    Tx_Buffer_Index ^= 1U;
  }

  return status;
}

void Class_Serialport_USB::Data_Process(const uint8_t *Rx_Data, uint16_t Length)
{
  for (uint16_t i = 0U; i < Length; i++)
  {
    const uint8_t byte = Rx_Data[i];

    // 同时兼容CR、LF和CRLF; 连续行结束符不会产生空指令.
    if (byte == '\r' || byte == '\n')
    {
      if (!Rx_Discard_Until_Line_End && Rx_Line_Length > 0U)
      {
        Parse_Assignment(Rx_Line_Buffer, Rx_Line_Length);
      }

      Rx_Line_Length = 0U;
      Rx_Discard_Until_Line_End = false;
      continue;
    }

    if (Rx_Discard_Until_Line_End)
    {
      continue;
    }

    // 文本协议不接受嵌入NUL; 丢弃本行并等待下一个行结束符恢复.
    if (byte == 0U)
    {
      Rx_Line_Length = 0U;
      Rx_Discard_Until_Line_End = true;
      continue;
    }

    if (Rx_Line_Length >= sizeof(Rx_Line_Buffer) - 1U)
    {
      Rx_Line_Length = 0U;
      Rx_Discard_Until_Line_End = true;
      continue;
    }

    Rx_Line_Buffer[Rx_Line_Length++] = byte;
  }
}

void Class_Serialport_USB::Parse_Assignment(const uint8_t *Line, uint16_t Length)
{
  uint16_t line_begin = 0U;
  uint16_t line_end = Length;

  while (line_begin < line_end && (Line[line_begin] == ' ' || Line[line_begin] == '\t'))
  {
    line_begin++;
  }
  while (line_end > line_begin && (Line[line_end - 1U] == ' ' || Line[line_end - 1U] == '\t'))
  {
    line_end--;
  }

  uint16_t equal_position = line_begin;
  while (equal_position < line_end && Line[equal_position] != '=')
  {
    equal_position++;
  }

  if (equal_position == line_end)
  {
    Variable_Index = -1;
    return;
  }

  uint16_t name_end = equal_position;
  while (name_end > line_begin && (Line[name_end - 1U] == ' ' || Line[name_end - 1U] == '\t'))
  {
    name_end--;
  }

  uint16_t value_begin = static_cast<uint16_t>(equal_position + 1U);
  while (value_begin < line_end && (Line[value_begin] == ' ' || Line[value_begin] == '\t'))
  {
    value_begin++;
  }

  if (name_end == line_begin || value_begin == line_end)
  {
    Variable_Index = -1;
    return;
  }

  const int8_t variable_index =
      Find_Variable(Line + line_begin,
                    static_cast<uint16_t>(name_end - line_begin));
  if (variable_index < 0)
  {
    Variable_Index = -1;
    return;
  }

  float variable_value = 0.0f;
  if (!Parse_Variable_Value(Line + value_begin,
                            static_cast<uint16_t>(line_end - value_begin),
                            &variable_value))
  {
    Variable_Index = -1;
    return;
  }

  Variable_Value = variable_value;
  Variable_Index = variable_index;
}

int8_t Class_Serialport_USB::Find_Variable(const uint8_t *Name,
                                           uint16_t Length) const
{
  if (Name == nullptr || Length == 0U || Rx_Variable_List == nullptr)
  {
    return -1;
  }

  const char *variable_list =
      reinterpret_cast<const char *>(Rx_Variable_List);

  for (uint8_t i = 0U; i < Rx_Variable_Num; i++)
  {
    const char *candidate =
        variable_list + i * SERIALPORT_USB_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH;

    uint16_t candidate_length = 0U;
    while (candidate_length < SERIALPORT_USB_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH && candidate[candidate_length] != '\0')
    {
      candidate_length++;
    }

    if (candidate_length == Length && memcmp(candidate, Name, Length) == 0)
    {
      return static_cast<int8_t>(i);
    }
  }

  return -1;
}

bool Class_Serialport_USB::Parse_Variable_Value(const uint8_t *Text,
                                                uint16_t Length,
                                                float *Value) const
{
  if (Text == nullptr || Value == nullptr || Length == 0U)
  {
    return false;
  }

  uint16_t position = 0U;
  float sign = 1.0f;

  if (Text[position] == '+' || Text[position] == '-')
  {
    if (Text[position] == '-')
    {
      sign = -1.0f;
    }
    position++;
  }

  if (position == Length)
  {
    return false;
  }

  float result = 0.0f;
  float decimal_scale = 0.1f;
  bool decimal_point_seen = false;
  bool digit_seen = false;

  for (; position < Length; position++)
  {
    const uint8_t byte = Text[position];

    if (byte == '.')
    {
      if (decimal_point_seen)
      {
        return false;
      }
      decimal_point_seen = true;
      continue;
    }

    if (byte < '0' || byte > '9')
    {
      return false;
    }

    digit_seen = true;
    const float digit = static_cast<float>(byte - '0');

    if (decimal_point_seen)
    {
      result += digit * decimal_scale;
      decimal_scale *= 0.1f;
    }
    else
    {
      result = result * 10.0f + digit;
      if (result > FLT_MAX)
      {
        return false;
      }
    }
  }

  if (!digit_seen)
  {
    return false;
  }

  *Value = result * sign;
  return true;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
