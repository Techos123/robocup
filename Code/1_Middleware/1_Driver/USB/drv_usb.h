/**
 * @file drv_usb.h
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
 * 本模块采用单缓冲区接收 + 运行时 Hook 方案:
 *
 * 单缓冲区:
 *   USB_Manage_Object 内部维护一个 Rx_Buffer[USB_BUFFER_SIZE],
 *   USB_Init() 时将该缓冲区注册到 USB CDC 协议栈,
 *   每次数据到达后, 回调函数直接处理该缓冲区.
 *
 * 运行时 Hook (零侵入):
 *   USB_Init() 在运行时替换 USBD_Interface_fops_FS.Receive 函数指针,
 *   将原有的 CDC_Receive_FS 替换为我们的 Hook 函数.
 *   Hook 函数中先处理数据, 再调用原始 Receive 重新装载缓冲区.
 *
 *   **完全不需要修改 usbd_cdc_if.c 的任何代码.**
 */

#ifndef DRV_USB_H
#define DRV_USB_H

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* Exported macros -----------------------------------------------------------*/

// 接收缓冲区字节长度
#define USB_BUFFER_SIZE 512

/* Exported types ------------------------------------------------------------*/

/**
 * @brief USB通信接收回调函数数据类型
 *
 */
typedef void (*USB_Callback)(uint8_t *Buffer, uint16_t Length);

/**
 * @brief USB通信处理结构体（单缓冲区版本）
 *
 * 相比H7版本的双缓冲方案，此版本只使用一个接收缓冲区:
 * - 数据到达后直接在回调中处理，无需缓冲区切换
 * - 内存占用减少一半(从2×512到1×512)
 * - 要求上层在回调中快速处理或拷贝数据
 */
struct Struct_USB_Manage_Object
{
    // 接收缓冲区（单缓冲区）
    uint8_t Rx_Buffer[USB_BUFFER_SIZE];

    // 用户注册的回调函数
    USB_Callback Callback_Function;

    // 接收时间戳 (ms, 基于 HAL_GetTick())
    uint64_t Rx_Time_Stamp;
};

/* Exported variables --------------------------------------------------------*/

extern struct Struct_USB_Manage_Object USB_Manage_Object;

/* Exported function declarations --------------------------------------------*/

void USB_Init(USB_Callback Callback_Function);

uint8_t USB_Transmit_Data(uint8_t *Data, uint16_t Length);

void USB_ReceiveCallback(uint16_t Size);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
