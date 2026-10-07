/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fdcan.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_power.h"
#include "crt_fw.h"
#include "crt_music.h"
#include "drv_can.h"
#include "drv_uart.h"
#include "drv_usb.h"
#include "dvc_ahrs_wheeltec.h"
#include "dvc_motion_protocol.h"
#include "dvc_motor_dji.h"
#include "sys_timestamp.h"

// #include "User_Code\2_Device\Serialplot\dvc_serialplot.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

enum Enum_Turn_State
{
  Turn_State_IDLE = 0,
  Turn_State_ACTIVE,
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Class_Serialplot serialplot;
Class_Motion_Protocol_USB ros;
Class_Chassis_FW car;
Class_Music music;
Class_AHRS_WHEELTEC wt;
float Yaw = 0.0f;
uint32_t Counter = 0;
volatile float V = 0.0f;
volatile float Angle_Deg = 0.0f;
volatile float Target_Angle = 0.0f;
volatile bool Turn_Flag = false;
volatile bool Stop_Request = false;
volatile Enum_Turn_State Turn_State = Turn_State_IDLE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void WT_UART_Call_Back(uint8_t *Buffer, uint16_t Length)
{
  wt.UART_RxCpltCallback(Buffer, Length);
}

static float Main_Get_Shortest_Yaw_Error(float target_yaw_deg,
                                         float current_yaw_deg)
{
  float error_deg = target_yaw_deg - current_yaw_deg;

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

static bool Main_Take_Stop_Request()
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const bool stop_requested = Stop_Request;
  Stop_Request = false;
  if (primask == 0U)
  {
    __enable_irq();
  }

  return stop_requested;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM5)
  {
    SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
  }
}

void ROS_Call_Back(const Struct_Motion_Protocol_Command &Command)
{
  switch (Command.Command)
  {
    case (Motion_Protocol_Command_VEL):
    {
      if (!Turn_Flag)
      {
        V = Command.Linear_X;
        // 保留现有8字节VEL协议，Angular_Z在角度闭环底盘中不参与控制。
      }

      break;
    }

    case (Motion_Protocol_Command_STOP):
    {
      V = 0.0f;
      Turn_Flag = false;
      Turn_State = Turn_State_IDLE;
      Stop_Request = true;

      break;
    }

    case (Motion_Protocol_Command_TURN):
    {
      if (Turn_State == Turn_State_IDLE && !Turn_Flag)
      {
        Angle_Deg = Command.Angle_Deg; // Command.Angle_Deg：相对当前方向，逆时针为正
        V = 0.0f;
        Turn_Flag = true;
      }
      break;
    }
    default:
      break;
  }
}

void CAN3_Motor_Call_Back(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
  switch (Header.Identifier)
  {
    case (0x201):
    {
      car.Motor_Wheel[0].CAN_RxCpltCallback();
      break;
    }
    case (0x202):
    {
      car.Motor_Wheel[1].CAN_RxCpltCallback();
      break;
    }
    case (0x203):
    {
      car.Motor_Wheel[2].CAN_RxCpltCallback();
      break;
    }
    case (0x204):
    {
      car.Motor_Wheel[3].CAN_RxCpltCallback();
      break;
    }
    default:
      break;
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART7_Init();
  MX_USB_DEVICE_Init();
  MX_FDCAN3_Init();
  MX_TIM5_Init();
  MX_USART10_UART_Init();
  MX_TIM12_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  BSP_Power.Init(false, false, true);
  SYS_Timestamp.Init(&htim5);
  HAL_TIM_Base_Start_IT(&htim5);

  UART_Init(&huart7, WT_UART_Call_Back, 128);
  wt.Init(&huart7);
  // 初始化校准完成后立即更新状态，避免首个100ms周期使用无效姿态。
  wt.TIM_100ms_Alive_PeriodElapsedCallback();

  car.Init();
  car.Set_Chassis_Yaw_Control_Type(Chassis_Yaw_Control_Type_ANGLE);

  CAN_Init(&hfdcan3, CAN3_Motor_Call_Back);

  ros.Init(ROS_Call_Back);

  music.Play_Startup();

  car.Set_Chassis_Control_Type(Chassis_Control_Type_NORMAL);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Struct_Chassis_Attitude_Feedback attitude_feedback{};
    if (wt.Get_Status() == AHRS_WHEELTEC_Status_ENABLE)
    {
      Yaw = wt.Get_Angle_Yaw();
      attitude_feedback.Yaw_Deg = Yaw;
      attitude_feedback.Yaw_Rate_Rad_S = wt.Get_Omega_Yaw();
      attitude_feedback.Valid = true;
    }
    car.Set_Attitude_Feedback(attitude_feedback);

    if (Main_Take_Stop_Request())
    {
      car.Set_Target_Velocity_X(0.0f);
      if (attitude_feedback.Valid)
      {
        Target_Angle = attitude_feedback.Yaw_Deg;
        car.Set_Target_Angle(Target_Angle);
      }
      car.Reset_Wheel_Omega_PID();
    }

    const bool turn_in_progress =
        (Turn_State == Turn_State_ACTIVE) || Turn_Flag;
    car.Set_Target_Velocity_X(turn_in_progress ? 0.0f : V);

    const uint32_t turn_start_primask = __get_PRIMASK();
    __disable_irq();
    if (Turn_State == Turn_State_IDLE &&
        Turn_Flag &&
        !Stop_Request &&
        attitude_feedback.Valid)
    {
      Target_Angle = Angle_Deg + attitude_feedback.Yaw_Deg;
      if (Target_Angle > 180.0f)
      {
        Target_Angle -= 360.0f;
      }
      else if (Target_Angle < -180.0f)
      {
        Target_Angle += 360.0f;
      }
      car.Set_Target_Angle(Target_Angle);
      Turn_State = Turn_State_ACTIVE;
      Turn_Flag = false;
    }
    if (turn_start_primask == 0U)
    {
      __enable_irq();
    }

    if (Counter % 2 == 0)
    {
      car.TIM_2ms_Resolution_PeriodElapsedCallback();
      car.TIM_2ms_Control_PeriodElapsedCallback();
    }

    float yaw_error_deg =
        Main_Get_Shortest_Yaw_Error(Target_Angle, attitude_feedback.Yaw_Deg);
    float current_yaw_rate_rad_s = attitude_feedback.Yaw_Rate_Rad_S;

    if (Turn_State == Turn_State_ACTIVE &&
        attitude_feedback.Valid &&
        fabsf(yaw_error_deg) <= 1.0f &&
        fabsf(current_yaw_rate_rad_s) <= 0.05f &&
        fabsf(car.Get_Now_Velocity_X()) <= 0.02f)
    {
      const uint32_t turn_done_primask = __get_PRIMASK();
      __disable_irq();
      if (Turn_State == Turn_State_ACTIVE && !Stop_Request)
      {
        const uint8_t status = ros.Send_Turn_Done();
        if (status == USBD_OK)
        {
          Turn_State = Turn_State_IDLE;
        }
      }
      if (turn_done_primask == 0U)
      {
        __enable_irq();
      }
    }
    // 通信设备回调数据
    TIM_1ms_CAN_PeriodElapsedCallback();

    Counter++;
    if (Counter >= 100)
    {
      Counter = 0;
      wt.TIM_100ms_Alive_PeriodElapsedCallback();
      car.TIM_100ms_Alive_PeriodElapsedCallback();
    }

    HAL_Delay(0);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
   */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
   */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2 | RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 40;
  PeriphClkInitStruct.PLL2.PLL2P = 6;
  PeriphClkInitStruct.PLL2.PLL2Q = 4;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state
   */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
