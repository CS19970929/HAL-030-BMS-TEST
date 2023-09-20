/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  // char tx_buf[10] = "hello!";
  char tx_buf[10] = "hello!";
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();
  MX_RTC_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C2_Init();
  MX_TIM15_Init();
  MX_TIM17_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  InitDevice(); 
  InitVar();    

  HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, sizeof(tx_buf) / sizeof(char), 1000);
  HAL_UART_Transmit(&huart2, (uint8_t *)tx_buf, sizeof(tx_buf) / sizeof(char), 1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // HAL_RTC_GetAlarm()
    // HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
    // HAL_IWDG_Refresh(&hiwdg);
    // HAL_Delay(500);

    // HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 2 */

void InitDevice(void)
{
	// SystemInit(); // 直接调用就可以了。

	Init_IAPAPP();

#if (defined _DEBUG_CODE)
	InitIO();
	InitDelay();
	InitTimer();
	InitSystemWakeUp();

#else
	IsSleepStartUp();

	// InitIO();
	// InitDelay();
	// InitTimer();
	InitSystemWakeUp();

  //todo conf
	InitE2PROM(); // 内部EEPROM，不需要初始化
	InitSci();
	InitADC();

	InitData_SOC();
	// Init_RTC(); // 必须放在EEPROM读完数据后面！
	InitHeat_Cool();
	InitMosRelay_DOx();
	Init_ChargerLoad_Det();

	// InitPWM();			//关于CBC的输出

	InitAFE1();

	Init_IWDG();
#endif
}

void InitVar(void)
{
	UINT16 i;

	// SystemMonitorResetData_EEPROM();							//这个函数的初始化默认需求功能修改了，要修改EEPROM的上电标志位
	InitSystemMonitorData_EEPROM();
	MCUO_RELAY_PRE = SystemStatus.bits.b1Status_Relay_PRE; // 选择，需要的在这里赋值
	MCUO_RELAY_MAIN = SystemStatus.bits.b1Status_Relay_MAIN;

	// Switch功能
	// Switch_OnOFF_Func.all = 0;
	// InitSwitchData_EEPROM();

	// 总系统错误监控系统初始化
	for (i = 0; i < ERROR_NUM; ++i)
	{
		//*(&System_ErrFlag.u8ErrFlag_Com_AFE1+i)  =  0;		//有病，这样写就把错误清除了
	}

	// 保护标志位初始化
	g_stCellInfoReport.unMdlFault_First.all = 0;
	g_stCellInfoReport.unMdlFault_Second.all = 0;
	g_stCellInfoReport.unMdlFault_Third.all = 0;
	// 当次保护记录初始化
	FaultPoint_First = 0;
	FaultPoint_Second = 0;
	FaultPoint_Third = 0;

	FaultPoint_First2 = 0;
	FaultPoint_Second2 = 0;
	FaultPoint_Third2 = 0;
	for (i = 0; i < Record_len; ++i)
	{
		Fault_record_First[i] = 0;
		Fault_record_Second[i] = 0;
		Fault_record_Third[i] = 0;

		Fault_record_First2[i] = 0;
		Fault_record_Second2[i] = 0;
		Fault_record_Third2[i] = 0;
	}
	Fault_Flag_Fisrt.all = 0;
	Fault_Flag_Second.all = 0;
	Fault_Flag_Third.all = 0;

	// 继电器驱动开启初始化							//不打算放在这里
	// RelayCtrl_Command = RELAY_PRE_DET;	//不打算放在这里
	// HeatCtrl_Command = ST_HEAT_DET_SELF;
	// CoolCtrl_Command = ST_COOL_DET_SELF;

	// 休眠相关

	// 这样写就不用管前面到底读出来还是复位了(在EEPROM很多个地方算)
	SeriesNum = OtherElement.u16Sys_SeriesNum;
	g_u32CS_Res_AFE = ((UINT32)OtherElement.u16Sys_CS_Res_Num * 844 << 10) / OtherElement.u16Sys_CS_Res / 100; // 算CS检流电阻

	LogRecord_Flag.bits.Log_StartUp = 1;
}
#if 0
void App_WakeUpAFE(void)
{
	MCUO_WAKEUP_AFE = 0;
	__delay_ms(1);
	MCUO_WAKEUP_AFE = 1;
	__delay_ms(5); // max 2ms，tBOOT_max
	MCUO_WAKEUP_AFE = 0;
	__delay_ms(10); // 自检，10ms，tBOOTREADY
}
#else

void App_WakeUpAFE(void)
{
  HAL_GPIO_WritePin(AFE_WAKEUP_GPIO_Port, AFE_WAKEUP_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);

  HAL_GPIO_WritePin(AFE_WAKEUP_GPIO_Port, AFE_WAKEUP_Pin, GPIO_PIN_SET);
  HAL_Delay(5);

  HAL_GPIO_WritePin(AFE_WAKEUP_GPIO_Port, AFE_WAKEUP_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
}
#endif

void InitSystemWakeUp(void)
{
	App_WakeUpAFE();

  //todo conf
	MCUO_BEL_EN = 1;

  HAL_GPIO_WritePin(AFE_ALARM_GPIO_Port, AFE_ALARM_Pin, GPIO_PIN_SET);

	// MCUO_PW_RS485_EN = 1;
  HAL_Delay(20);
}
  /* USER CODE END 2 */

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI14 | RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /* User can add his own implementation to report the HAL error return state */
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
