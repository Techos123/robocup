/**
 * @file drv_usb.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的USB通信初始化与配置流程
 * @version 2.2
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2025-08-13 1.1 适配达妙MC02板(H7双缓冲)
 * @date 2026-06-21 2.1 适配C板(F4), 简化为单缓冲区接收
 * @date 2026-06-21 2.2 运行时 Hook USBD_Interface_fops_FS.Receive, 无需修改 usbd_cdc_if.c
 *
 * @copyright USTC-RoboWalker (c) 2023-2026
 *
 */

/**
 * 运行时 Hook 方案说明:
 *
 * 核心思路: USBD_Interface_fops_FS 不是 const 的,
 *           其 Receive 函数指针可以在运行时被替换.
 *
 * 调用链对比:
 *
 *   默认 (CubeMX):
 *     USB中断 → usbd_cdc.c → fops->Receive → CDC_Receive_FS (usbd_cdc_if.c)
 *                              (SetRxBuffer + ReceivePacket, 仅重新装载)
 *
 *   本模块 Hook 后:
 *     USB中断 → usbd_cdc.c → fops->Receive → USB_Hook_Receive (drv_usb.cpp)
 *                                                ├─ USB_ReceiveCallback(Size)
 *                                                │    ├─ 记录时间戳
 *                                                │    └─ 用户回调函数(Buf, Size)
 *                                                └─ Original_CDC_Receive_FS(Buf, Len)
 *                                                     └─ SetRxBuffer + ReceivePacket (重新装载)
 *
 *   优势:
 *     - 完全不需要修改 usbd_cdc_if.c
 *     - CubeMX 重新生成代码时不会覆盖我们的逻辑
 *     - Hook 是确定性的, 无竞态问题
 *
 * 单缓冲区 vs 双缓冲:
 *
 *   双缓冲方案 (H7参考):
 *     维护两个独立缓冲区, 一个给USB硬件填充, 另一个给上层处理,
 *     每次接收后交换指针. 适合高频大数据量场景.
 *
 *   单缓冲方案 (本实现):
 *     只维护一个缓冲区, USB硬件直接填充,
 *     数据到达后立即在回调中处理.
 *     优点: 内存减半, 逻辑简单
 *     缺点: 处理期间USB无法写入, 需快速处理或自行拷贝数据
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_usb.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// USB通信管理对象实例
struct Struct_USB_Manage_Object USB_Manage_Object = {0};

// USB设备句柄, 由 usb_device.c 中定义
extern USBD_HandleTypeDef hUsbDeviceFS;

// CDC接口回调表, 由 usbd_cdc_if.c 中定义 (非const, 可运行时修改)
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

// 保存原始的 CDC_Receive_FS 函数指针
static int8_t (*Original_CDC_Receive_FS)(uint8_t *Buf, uint32_t *Len) = nullptr;

/* Private function declarations ---------------------------------------------*/

/**
 * @brief USB CDC Receive Hook 函数
 *
 * 替换 USBD_Interface_fops_FS.Receive, 在每次USB数据到达时被调用.
 * 先处理接收到的数据, 再调用原始的 Receive 函数重新装载缓冲区.
 *
 * @param Buf 接收缓冲区指针 (指向 USB_Manage_Object.Rx_Buffer)
 * @param Len 接收到的数据长度 (字节)
 * @return int8_t USBD_OK
 */
static int8_t USB_Hook_Receive(uint8_t *Buf, uint32_t *Len)
{
    // 1. 处理接收到的数据 — 直接使用 Buf (USB硬件实际填充的缓冲区)
    USB_Manage_Object.Rx_Time_Stamp = HAL_GetTick();
    if (USB_Manage_Object.Callback_Function != nullptr)
    {
        USB_Manage_Object.Callback_Function(Buf, (uint16_t)(*Len));
    }

    // 2. 调用原始 Receive 函数, 重新装载缓冲区并准备接收下一帧
    return Original_CDC_Receive_FS(Buf, Len);
}

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化USB通信
 *
 * @param Callback_Function 接收数据处理回调函数
 *
 * @note 调用时机: 在 MX_USB_DEVICE_Init() 之后调用
 * @note 此函数会做三件事:
 *         1. 保存原始 CDC_Receive_FS 函数指针
 *         2. 运行时 Hook USBD_Interface_fops_FS.Receive → USB_Hook_Receive
 *         3. 将本模块的 Rx_Buffer 注册到 USB CDC 协议栈并启动接收
 * @note 完全不需要修改 usbd_cdc_if.c
 */
void USB_Init(USB_Callback Callback_Function)
{
    USB_Manage_Object.Callback_Function = Callback_Function;

    // -------- 运行时 Hook --------
    // 保存原始的 Receive 函数指针 (CDC_Receive_FS)
    if (Original_CDC_Receive_FS == nullptr)
    {
        Original_CDC_Receive_FS = USBD_Interface_fops_FS.Receive;

        // 替换为我们的 Hook 函数
        USBD_Interface_fops_FS.Receive = USB_Hook_Receive;
    }

    // -------- 启动接收 --------
    // 将本模块的单缓冲区注册到 USB CDC 协议栈
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, USB_Manage_Object.Rx_Buffer);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
}

/**
 * @brief 通过USB发送数据帧
 *
 * @param Data 待发送数据指针
 * @param Length 数据长度 (字节)
 * @return uint8_t USBD_OK(0) 成功, USBD_BUSY(1) 忙碌, USBD_FAIL(2) 失败
 *
 * @note 内部调用 CubeMX 生成的 CDC_Transmit_FS()
 */
uint8_t USB_Transmit_Data(uint8_t *Data, uint16_t Length)
{
    return (CDC_Transmit_FS(Data, Length));
}

/**
 * @brief USB接收回调函数 (单缓冲区版本)
 *
 * @param Size 本次接收到的数据长度 (字节)
 *
 * @note 此函数由 USB_Hook_Receive() 调用
 * @note 单缓冲区模式下, 数据存放在 USB_Manage_Object.Rx_Buffer 中
 * @note 本函数会记录接收时间戳, 然后调用用户注册的回调函数
 */
void USB_ReceiveCallback(uint16_t Size)
{
    // 记录接收时间戳
    USB_Manage_Object.Rx_Time_Stamp = HAL_GetTick();

    // 调用用户注册的回调函数
    if (USB_Manage_Object.Callback_Function != nullptr)
    {
        USB_Manage_Object.Callback_Function(USB_Manage_Object.Rx_Buffer, Size);
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
