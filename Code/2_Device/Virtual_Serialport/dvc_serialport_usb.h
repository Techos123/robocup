/**
 * @file Serialport_USB.cpp
 * @author Techos (3069104675@qq.com)
 * @brief USB CDC虚拟串口下的JustFloat发送与文本变量赋值接收封装
 * @version 0.2
 *
 * @date 2026-8-7  完成USB CDC虚拟串口下的JustFloat发送与文本变量赋值接收封装
 * @copyright SUST-RoboZB 2026
 */

#ifndef SERIALPORT_USB_H
#define SERIALPORT_USB_H

/* Includes ------------------------------------------------------------------*/

#include "drv_usb.h"

#include <type_traits>

/* Exported macros -----------------------------------------------------------*/

// 文本协议单条指令缓冲区长度, 包含末尾字符串结束符
#define SERIALPORT_USB_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH 100U

// JustFloat最大发送通道数
#define SERIALPORT_USB_TX_CHANNEL_MAX 25U

/* Exported types ------------------------------------------------------------*/

typedef float (*Serialport_USB_Data_Reader)(const void *Address);

/**
 * @brief USB CDC虚拟串口封装
 *
 * 发送协议: float数组(小端) + 0x00 0x00 0x80 0x7f
 * 接收协议: "variable=value\r"
 *
 * USB CDC是字节流, USB_RxCpltCallback()会按行结束符组装文本指令,
 * 因此支持一条指令跨多个USB数据包以及一个数据包包含多条指令.
 */
class Class_Serialport_USB
{
public:
  /**
   * @brief 初始化USB虚拟串口封装
   *
   * @param Callback_Function USB接收完成回调函数
   * @param __Rx_Variable_Assignment_Num 接收变量字典数量
   * @param __Rx_Variable_Assignment_List 固定宽度二维变量字典首地址,
   *        每个元素占SERIALPORT_USB_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH字节
   */
  void Init(USB_Callback Callback_Function,
            uint8_t __Rx_Variable_Assignment_Num = 0,
            char **__Rx_Variable_Assignment_List = nullptr);

  inline int8_t Get_Variable_Index() const;

  inline float Get_Variable_Value() const;

  template <typename... Args>
  inline void Set_Data(const Args &...args);

  void USB_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);

  uint8_t JustFloat_Output();

protected:
  // 固定宽度二维变量字典, 与参考Serialplot接口保持兼容
  uint8_t Rx_Variable_Num = 0;
  char **Rx_Variable_List = nullptr;

  // JustFloat通道可以绑定数值地址, 也可以保存调用Set_Data()时传入的值
  const void *Data_Address[SERIALPORT_USB_TX_CHANNEL_MAX] = {};
  Serialport_USB_Data_Reader Data_Reader[SERIALPORT_USB_TX_CHANNEL_MAX] = {};
  float Data_Value[SERIALPORT_USB_TX_CHANNEL_MAX] = {};
  uint8_t Data_Number = 0;

  // 双发送工作缓冲区: 防止CDC异步发送期间覆盖正在发送的数据
  uint8_t Tx_Buffer[2][USB_BUFFER_SIZE] = {};
  uint8_t Tx_Buffer_Index = 0;

  // CDC字节流文本组包状态
  uint8_t Rx_Line_Buffer[SERIALPORT_USB_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = {};
  uint16_t Rx_Line_Length = 0;
  bool Rx_Discard_Until_Line_End = false;

  // 最近一条有效变量赋值
  int8_t Variable_Index = -1;
  float Variable_Value = 0.0f;

  void Data_Process(const uint8_t *Rx_Data, uint16_t Length);

  void Parse_Assignment(const uint8_t *Line, uint16_t Length);

  int8_t Find_Variable(const uint8_t *Name, uint16_t Length) const;

  bool Parse_Variable_Value(const uint8_t *Text,
                            uint16_t Length,
                            float *Value) const;

  template <typename T>
  inline void Set_Data_Channel(uint8_t Index, const T &Value);
};

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取最近一条有效指令在变量字典中的编号
 *
 * @return int8_t 字典编号, -1表示最近一条完整指令无效或未匹配
 */
inline int8_t Class_Serialport_USB::Get_Variable_Index() const
{
  return Variable_Index;
}

/**
 * @brief 获取最近一条有效变量赋值
 */
inline float Class_Serialport_USB::Get_Variable_Value() const
{
  return Variable_Value;
}

/**
 * @brief 注册JustFloat发送通道
 *
 * 每个参数可以是算术类型数值或指向算术类型的指针:
 * - 数值会在调用Set_Data()时转换为float并保存;
 * - 指针会被注册, 每次JustFloat_Output()时读取其最新值.
 */
template <typename... Args>
inline void Class_Serialport_USB::Set_Data(const Args &...args)
{
  static_assert(sizeof...(Args) > 0,
                "Class_Serialport_USB::Set_Data requires at least one channel");
  static_assert(sizeof...(Args) <= SERIALPORT_USB_TX_CHANNEL_MAX,
                "Class_Serialport_USB::Set_Data exceeds SERIALPORT_USB_TX_CHANNEL_MAX");

  const uint8_t count = static_cast<uint8_t>(sizeof...(Args));
  uint8_t index = 0U;
  (Set_Data_Channel(index++, args), ...);
  Data_Number = count;
}

template <typename T>
inline void Class_Serialport_USB::Set_Data_Channel(uint8_t Index, const T &Value)
{
  using Decayed_Type = typename std::decay<T>::type;

  if constexpr (std::is_pointer<Decayed_Type>::value)
  {
    using Pointed_Type = typename std::remove_cv<
        typename std::remove_pointer<Decayed_Type>::type>::type;
    static_assert(std::is_arithmetic<Pointed_Type>::value,
                  "Class_Serialport_USB pointer channels must point to arithmetic data");

    Data_Address[Index] = static_cast<const void *>(Value);
    Data_Reader[Index] = [](const void *Address) -> float
    {
      return static_cast<float>(*static_cast<const Pointed_Type *>(Address));
    };
    Data_Value[Index] = 0.0f;
  }
  else
  {
    static_assert(std::is_arithmetic<Decayed_Type>::value,
                  "Class_Serialport_USB value channels must be arithmetic data");

    Data_Address[Index] = nullptr;
    Data_Reader[Index] = nullptr;
    Data_Value[Index] = static_cast<float>(Value);
  }
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
