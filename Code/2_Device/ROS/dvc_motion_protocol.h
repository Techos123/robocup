/**
 * @file dvc_motion_protocol.h
 * @brief 基于USB CDC虚拟串口的X5运动控制二进制协议封装
 * @version 2.1
 * @date 2026-10-11
 */

#ifndef DVC_MOTION_PROTOCOL_H
#define DVC_MOTION_PROTOCOL_H

/* 包含文件 ------------------------------------------------------------------*/

#include "drv_usb.h"

#include <stdint.h>

/* 公有宏 --------------------------------------------------------------------*/

// 固定帧头的第一个字节
#define MOTION_PROTOCOL_FRAME_HEADER_0 0xAAU
// 固定帧头的第二个字节
#define MOTION_PROTOCOL_FRAME_HEADER_1 0x55U
// 帧头、命令字、数据长度和CRC共占6字节
#define MOTION_PROTOCOL_FRAME_OVERHEAD 6U
// LEN字段为uint8_t，因此DATA区最大长度为255字节
#define MOTION_PROTOCOL_MAX_DATA_LENGTH 255U
// 协议允许的最大完整帧长度：255字节DATA加6字节固定字段
#define MOTION_PROTOCOL_MAX_FRAME_LENGTH \
  (MOTION_PROTOCOL_MAX_DATA_LENGTH + MOTION_PROTOCOL_FRAME_OVERHEAD)

/* 公有类型 ------------------------------------------------------------------*/

/**
 * @brief 运动协议命令字
 *
 * 命令方向严格遵循MOTION_PROTOCOL.md：
 * - VEL、STOP、TURN和START由X5发送给运动下位机；
 * - TURN_DONE由运动下位机发送给X5；
 * - NONE只表示本地对象尚未保存有效命令，不会在线上传输。
 */
enum Enum_Motion_Protocol_Command : uint8_t
{
  Motion_Protocol_Command_NONE = 0x00U,
  Motion_Protocol_Command_VEL = 0x01U,
  Motion_Protocol_Command_STOP = 0x02U,
  Motion_Protocol_Command_TURN = 0x03U,
  Motion_Protocol_Command_START = 0x04U,
  Motion_Protocol_Command_TURN_DONE = 0x83U,
};

/**
 * @brief 从X5收到并通过完整协议校验的一条运动命令
 *
 * 只有与Command对应的数据字段才有意义：
 * - VEL：Linear_X为底盘x轴线速度，单位m/s，向前为正；
 *   Angular_Z为底盘z轴角速度，单位rad/s，逆时针为正；
 * - STOP、START：没有DATA，三个浮点字段均保持为0；
 * - TURN：Angle_Deg为相对当前车身方向的目标转角，单位deg，
 *   逆时针左转为正、顺时针右转为负，有效范围为[-130, 130]。
 *
 * 结构体由协议对象在分发有效帧时临时创建。应用层只能在回调执行期间
 * 读取其内容；如需稍后处理，必须在回调内复制所需字段。
 */
struct Struct_Motion_Protocol_Command
{
  Enum_Motion_Protocol_Command Command = Motion_Protocol_Command_NONE;
  float Linear_X = 0.0f;
  float Angular_Z = 0.0f;
  float Angle_Deg = 0.0f;
};

/**
 * @brief 协议接收统计信息
 *
 * 计数器用于调试通信质量，不参与运动控制：
 * - Valid_Frame_Count：命令、长度、CRC和浮点数据全部合法的帧数；
 * - CRC_Error_Count：帧头和长度完整，但CRC校验失败的候选帧数；
 * - Format_Error_Count：CRC正确但命令未知、长度不匹配或浮点数非法的帧数。
 *
 * 计数器为uint32_t，长期运行溢出后会自然回绕到0。
 */
struct Struct_Motion_Protocol_Statistics
{
  uint32_t Valid_Frame_Count = 0U;
  uint32_t CRC_Error_Count = 0U;
  uint32_t Format_Error_Count = 0U;
};

/**
 * @brief 有效运动命令回调函数类型
 *
 * 该回调在USB接收上下文中同步执行。现有USB驱动会等回调返回后才重新挂载
 * 接收缓冲区，因此回调内不得调用延时函数、执行阻塞操作或进行耗时计算。
 * Command引用只在本次回调期间有效，不能保存该引用供回调返回后使用。
 */
typedef void (*Motion_Protocol_Command_Callback)(
    const Struct_Motion_Protocol_Command &Command);

/**
 * @brief 基于现有USB CDC驱动的运动协议V2.1收发类
 *
 * USB CDC向上层提供连续字节流，单次USB接收并不保证恰好对应一帧。本类使用
 * 固定长度缓冲区保存未处理字节，支持以下情况：
 * - 一帧被拆分到多次USB接收中的半包；
 * - 一次USB接收中连续包含多帧的粘包；
 * - 合法帧之前存在无关噪声字节；
 * - CRC错误后从当前帧头的第二个字节开始重新搜索下一组帧头。
 *
 * 现有USB驱动只提供一个全局接收回调，因此系统中只能初始化一个本类实例。
 * Init()会接管USB驱动的上层接收回调，不能与文本虚拟串口协议同时使用。
 */
class Class_Motion_Protocol_USB
{
public:
  /**
   * @brief 初始化协议状态并注册USB接收回调
   *
   * @param Callback_Function 有效VEL、STOP、TURN或START命令的应用层回调；
   *        允许传入nullptr，此时仍会解析并统计帧，但不会向应用层分发命令。
   *
   * @note 必须在MX_USB_DEVICE_Init()之后调用。重复调用会清空未完成帧和统计
   *       数据，并将USB接收数据转交给最后一次初始化的实例。
   */
  void Init(Motion_Protocol_Command_Callback Callback_Function);

  /**
   * @brief 处理一段由USB CDC接收到的原始字节流
   *
   * @param Rx_Data 原始接收数据首地址
   * @param Length 本次接收的数据长度，单位字节
   *
   * 本函数会逐字节追加数据并尝试解析所有完整帧。正常使用时由内部USB适配
   * 回调自动调用；保留为公有接口便于上层主动转交数据或进行协议测试。
   */
  void USB_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);

  /**
   * @brief 发送转向完成回执TURN_DONE
   *
   * 固定发送“AA 55 83 00 60 80”。只有底盘确认目标转角已经完成，并且车体
   * 已经进入稳定停止状态后才能调用。本函数不会自行判断转向是否完成。
   *
   * @return uint8_t USBD_OK表示已提交发送，USBD_BUSY表示USB正在发送上一帧，
   *         USBD_FAIL表示USB尚未就绪或发送参数无效
   */
  uint8_t Send_Turn_Done();

  /**
   * @brief 计算CRC-16/Modbus校验值
   *
   * @param Data 待校验数据首地址；协议帧中应指向CMD字段
   * @param Length 待校验长度；协议帧中应为CMD、LEN和DATA的总长度
   * @return uint16_t CRC计算结果，发送时需要先发送低字节、再发送高字节
   *
   * 算法初始值为0xFFFF，多项式为0xA001，最终异或值为0。帧头AA 55不参与
   * CRC计算。Data为nullptr且Length非0时返回0，避免访问无效地址。
   */
  static uint16_t Calculate_CRC16_Modbus(const uint8_t *Data,
                                         uint16_t Length);

  /**
   * @brief 获取当前协议接收统计信息
   *
   * @return const Struct_Motion_Protocol_Statistics& 统计对象的只读引用
   */
  inline const Struct_Motion_Protocol_Statistics &Get_Statistics() const;

private:
  // 当前接管USB驱动回调的协议实例；USB驱动只有一个全局接收通道
  static Class_Motion_Protocol_USB *Active_Instance;

  // 每条有效输入命令完成校验后调用的应用层回调
  Motion_Protocol_Command_Callback Command_Callback_Function = nullptr;

  // 保存一个最大候选帧及跨USB数据包遗留的未完成帧数据
  uint8_t Rx_Frame_Buffer[MOTION_PROTOCOL_MAX_FRAME_LENGTH] = {};
  // Rx_Frame_Buffer中当前有效数据的字节数
  uint16_t Rx_Frame_Length = 0U;

  // TURN_DONE为固定6字节帧；缓冲区必须长期存在以满足USB异步发送要求
  uint8_t Tx_Turn_Done_Frame[MOTION_PROTOCOL_FRAME_OVERHEAD] = {
      MOTION_PROTOCOL_FRAME_HEADER_0,
      MOTION_PROTOCOL_FRAME_HEADER_1,
      Motion_Protocol_Command_TURN_DONE,
      0x00U,
      0x60U,
      0x80U,
  };

  // 记录有效帧、CRC错误帧和格式错误帧的累计数量
  Struct_Motion_Protocol_Statistics Statistics = {};

  // 将USB驱动使用的普通函数指针转发到当前协议对象
  static void USB_Rx_Adapter(uint8_t *Rx_Data, uint16_t Length);

  // 搜索帧头、检查完整长度、校验CRC并持续处理缓冲区中的所有完整帧
  void Process_Rx_Buffer();

  // 严格检查命令对应的LEN和浮点数据，并向应用层分发有效命令
  bool Dispatch_Frame(uint8_t Command,
                      const uint8_t *Data,
                      uint8_t Data_Length);

  // 从接收缓冲区头部移除指定字节数，并将剩余数据前移
  void Remove_Leading_Bytes(uint16_t Length);

  // 按小端字节序解码IEEE-754 float32，同时拒绝NaN和正负无穷大
  static bool Decode_Float32_LE(const uint8_t *Data, float *Value);
};

/* 公有内联函数 --------------------------------------------------------------*/

inline const Struct_Motion_Protocol_Statistics &
Class_Motion_Protocol_USB::Get_Statistics() const
{
  return Statistics;
}

#endif
