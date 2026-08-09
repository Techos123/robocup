/**
 * @file bsp_power.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 板载电源, 包含两个24V和一个5V输出
 * @version 0.1
 * @date 2025-08-15 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_power.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Class_Power BSP_Power;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化电源
 *
 * @param dc24_0_status DC24V_0输出
 * @param dc24_1_status DC24V_1输出
 * @param dc5_status DC5V输出
 */
void Class_Power::Init(bool dc24_0_status, bool dc24_1_status, bool dc5_status)
{
    // ADC_Manage_Object = &ADC1_Manage_Object;

    Set_DC24_0(dc24_0_status);
    Set_DC24_1(dc24_1_status);
    Set_DC5(dc5_status);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
