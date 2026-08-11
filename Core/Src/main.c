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
#include "alg_basic.h"
#include "bsp_bmi088.h"
#include "crt_fw.h"
#include "drv_can.h"
#include "drv_spi.h"
#include "drv_usb.h"
#include "dvc_motor_dji.h"
#include "dvc_serialport_usb.h"
#include "sys_timestamp.h"

// #include "User_Code\2_Device\Serialplot\dvc_serialplot.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
Class_Serialport_USB serialport_usb;
Class_Chassis_FW car;
float Yaw, Pitch, Roll;
bool BMI088_Initialized = false;
uint32_t Last_USB_Output_Tick = 0;
uint32_t Counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SPI2_BMI088_Call_Back(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length)
{
  (void)Tx_Buffer;
  (void)Rx_Buffer;
  (void)Tx_Length;
  (void)Rx_Length;
  BSP_BMI088.SPI_RxCpltCallback();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (BMI088_Initialized &&
      (GPIO_Pin == BMI088_ACCEL__INTERRUPT_Pin || GPIO_Pin == BMI088_GYRO__INTERRUPT_Pin))
  {
    BSP_BMI088.EXTI_Flag_Callback(GPIO_Pin);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM5)
  {
    SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
  }
  else if (htim->Instance == TIM4)
  {
    BSP_BMI088.TIM_10us_Calculate_PeriodElapsedCallback();
  }
  else if (htim->Instance == TIM8)
  {
    BSP_BMI088.TIM_125us_Calculate_PeriodElapsedCallback();
  }
}

/**
 * @brief HAL库UART接收上位机消息DMA空闲中断
 *
 * @param huart UART编号
 * @param Size 长度
 */
void Serialport_USB_Call_Back(uint8_t *Buffer, uint16_t Length)
{
  serialport_usb.USB_RxCpltCallback(Buffer, Length);
  switch (serialport_usb.Get_Variable_Index())
  {
    // case (0):
    // {
    //   motor_3508_0x201.PID_Omega.Set_K_P(serialport_usb.Get_Variable_Value());
    //   break;
    // }
    // case (1):
    // {
    //   motor_3508_0x201.PID_Omega.Set_K_I(serialport_usb.Get_Variable_Value());
    //   break;
    // }
    // case (2):
    // {
    //   motor_3508_0x201.PID_Omega.Set_K_D(serialport_usb.Get_Variable_Value());
    //   break;
    // }
    // case (3):
    // {
    //   Target_Omega = serialport_usb.Get_Variable_Value();
    //   break;
    // }
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

  serialport_usb.Init(Serialport_USB_Call_Back, 0, nullptr);
  SPI_Init(&hspi2, SPI2_BMI088_Call_Back);
  BMI088_Initialized = BSP_BMI088.Init();
  if (BMI088_Initialized)
  {
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim8);
  }
  Last_USB_Output_Tick = HAL_GetTick();

  car.Init();
  car.Set_Chassis_Yaw_Control_Type(Chassis_Yaw_Control_Type_ANGLE);

  CAN_Init(&hfdcan3, CAN3_Motor_Call_Back);

  car.Set_Chassis_Control_Type(Chassis_Control_Type_NORMAL);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    const uint32_t now_tick = HAL_GetTick();
    if (BMI088_Initialized && (uint32_t)(now_tick - Last_USB_Output_Tick) >= 100)
    {
      Last_USB_Output_Tick = now_tick;
      const Class_Matrix_f32<3, 1> euler_angle = BSP_BMI088.Get_Euler_Angle();
      Yaw = euler_angle[0][0] / BASIC_MATH_DEG_TO_RAD;
      Pitch = euler_angle[1][0] / BASIC_MATH_DEG_TO_RAD;
      Roll = euler_angle[2][0] / BASIC_MATH_DEG_TO_RAD;

      serialport_usb.Set_Data(&Yaw, &Pitch, &Roll);
      serialport_usb.JustFloat_Output();
    }

    Struct_Chassis_Attitude_Feedback attitude_feedback;
    if (BMI088_Initialized && BSP_BMI088.Get_Attitude_Valid())
    {
      const Class_Matrix_f32<3, 1> chassis_euler_angle = BSP_BMI088.Get_Euler_Angle();
      const Class_Matrix_f32<3, 1> chassis_gyro_body = BSP_BMI088.Get_Gyro_Body();
      attitude_feedback.Yaw_Deg = chassis_euler_angle[0][0] / BASIC_MATH_DEG_TO_RAD;
      attitude_feedback.Yaw_Rate_Rad_S = chassis_gyro_body[2][0];
      attitude_feedback.Valid = true;
    }
    car.Set_Attitude_Feedback(attitude_feedback);

    car.Set_Target_Velocity_X(0.5f);
    if (Counter % 2 == 0)
    {
      car.TIM_2ms_Resolution_PeriodElapsedCallback();
      car.TIM_2ms_Control_PeriodElapsedCallback();
    }

    // 通信设备回调数据
    TIM_1ms_CAN_PeriodElapsedCallback();

    Counter++;
    if (Counter >= 100)
    {
      Counter = 0;
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
