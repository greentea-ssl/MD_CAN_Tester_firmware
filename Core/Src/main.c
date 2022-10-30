/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "ntshell.h"
#include "usrcmd.h"

#include "usbd_cdc_if.h"


#include "motorTest.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define uart_puts(str) puts(str)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


ntshell_t nts;

MotorTest_TypeDef motorTest;

/********** LED Control **********/

volatile uint32_t LED_blink_count = 0;
volatile uint32_t LED_blink_state = 0;
volatile uint32_t LED_blink_t_us = 0;
volatile uint32_t LED_blink_times = 0;
volatile uint32_t LED_blink_Ton_us = 50000;
volatile uint32_t LED_blink_Toff_us = 200000;
volatile uint32_t LED_blink_T_wait_us = 1000000;
volatile uint32_t LED_blink_Ts_us = 1000;





int16_t Iq_res_int16 = 0;
uint16_t theta_res_uint16 = 0;
int16_t omega_res_int16 = 0;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

int32_t printf_cdc(char *format, ...);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
void __io_putchar(uint8_t ch)
{
	HAL_UART_Transmit(&huart2, &ch, 1, 1);
}



void driveMotor_speed(float Iq_ref[])
{

	uint32_t can1TxMailbox;
	CAN_TxHeaderTypeDef can1TxHeader;
	uint8_t can1TxData[8];

	int16_t Iq_ref_int[4];


	can1TxHeader.StdId = 0x300;
	can1TxHeader.ExtId = 0x00;
	can1TxHeader.IDE = CAN_ID_STD;
	can1TxHeader.RTR = CAN_RTR_DATA;
	can1TxHeader.DLC = 8;

	Iq_ref_int[0] = (int16_t)(Iq_ref[0] * 1024);
	Iq_ref_int[1] = (int16_t)(Iq_ref[1] * 1024);
	Iq_ref_int[2] = (int16_t)(Iq_ref[2] * 1024);
	Iq_ref_int[3] = (int16_t)(Iq_ref[3] * 1024);

	can1TxData[0] = Iq_ref_int[0] & 0xff;
	can1TxData[1] = (Iq_ref_int[0] >> 8) & 0xff;

	can1TxData[2] = Iq_ref_int[1] & 0xff;
	can1TxData[3] = (Iq_ref_int[1] >> 8) & 0xff;

	can1TxData[4] = Iq_ref_int[2] & 0xff;
	can1TxData[5] = (Iq_ref_int[2] >> 8) & 0xff;

	can1TxData[6] = Iq_ref_int[3] & 0xff;
	can1TxData[7] = (Iq_ref_int[3] >> 8) & 0xff;

	//HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);

	HAL_CAN_AddTxMessage(&hcan1, &can1TxHeader, can1TxData, &can1TxMailbox);

	return;
}



void requestMotorParam()
{

	uint32_t can1TxMailbox;
	CAN_TxHeaderTypeDef can1TxHeader;
	uint8_t can1TxData[8];

	can1TxHeader.StdId = 0x310;
	can1TxHeader.ExtId = 0x00;
	can1TxHeader.IDE = CAN_ID_STD;
	can1TxHeader.RTR = CAN_RTR_DATA;
	can1TxHeader.DLC = 0;

	//HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);

	HAL_CAN_AddTxMessage(&hcan1, &can1TxHeader, can1TxData, &can1TxMailbox);

	return;
}


uint8_t getChannel()
{
	uint8_t ch = 0;

	//ch |= !HAL_GPIO_ReadPin(CH_1_GPIO_Port, CH_1_Pin) << 2;
	ch |= !HAL_GPIO_ReadPin(CH_2_GPIO_Port, CH_2_Pin) << 1;
	ch |= !HAL_GPIO_ReadPin(CH_3_GPIO_Port, CH_3_Pin) << 0;
	//ch |= !HAL_GPIO_ReadPin(CH_4_GPIO_Port, CH_4_Pin) << 3;

	return ch;
}


float getVolume()
{
	static float rate = 0.0;

#if 0
	if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
	{
		Error_Handler();
	}
	else
	{
		rate = HAL_ADC_GetValue(&hadc1) / 4096.0f;
	}
#else
	rate = HAL_ADC_GetValue(&hadc1) / 4096.0f;
#endif

	if(HAL_ADC_Start_IT(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	return rate;

}

int32_t printf_cdc(char *format, ...)
{
	int32_t TransStrLength;
	static char TransStr[1024];

	va_list args;
	va_start(args, format);
	TransStrLength = vsprintf(TransStr, format, args);
	va_end(args);

	CDC_Transmit_FS((uint8_t *)TransStr, TransStrLength);

	return TransStrLength;
}


void LED_blink()
{

	switch(LED_blink_state)
	{
	case 0: // OFF WAIT
		HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);
		if(LED_blink_t_us >= LED_blink_T_wait_us)
		{
			if(LED_blink_times > 0)
			{
				LED_blink_state = 1;
				LED_blink_count = 0;
			}

			LED_blink_t_us = 0;
		}
		break;

	case 1: // ON
		HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
		if(LED_blink_t_us >= LED_blink_Ton_us)
		{
			LED_blink_count += 1;
			LED_blink_state = 2;
			LED_blink_t_us = 0;
		}
		break;

	case 2: // OFF
		HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);
		if(LED_blink_t_us >= LED_blink_Toff_us)
		{
			if(LED_blink_count < LED_blink_times)
				LED_blink_state = 1;
			else
				LED_blink_state = 0;

			LED_blink_t_us = 0;
		}
		break;

	default:

		break;
	}

	LED_blink_t_us += LED_blink_Ts_us;

}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim)
{

	if(htim->Instance == TIM1)
	{
		// 1ms
		LED_blink();

		motorTest.volume = getVolume();

		MotorTest_Update(&motorTest);


	}

}




void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	CAN_RxHeaderTypeDef can1RxHeader;
	uint8_t can1RxData[8];

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can1RxHeader, can1RxData);

	if(can1RxHeader.StdId == (0x400 + getChannel()) && can1RxHeader.DLC == 8)
	{
		motorTest.status_code = can1RxData[0];
		motorTest.Iq_res_int16 = ((int16_t)can1RxData[2] << 8) | can1RxData[1];
		motorTest.theta_res_uint16 = ((uint16_t)can1RxData[4] << 8) | can1RxData[3];
		motorTest.omega_res_int16 = ((int16_t)can1RxData[6] << 8) | can1RxData[5];
		return;
	}
	if(can1RxHeader.StdId == (0x410 + getChannel()) && can1RxHeader.DLC == 4)
	{
		uint16_t motor_kv = ((int16_t)can1RxData[1] << 8) | can1RxData[0];
		uint16_t Irated_uint16 = ((uint16_t)can1RxData[3] << 8) | can1RxData[2];
		float Irated = Irated_uint16 / 1024.0f;
		printf("ch = %d\n", can1RxHeader.StdId & 0x003);
		printf("KV = %d\n", motor_kv);
		printf("Irated = %f\n", Irated);
		return;
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


	CAN_FilterTypeDef sFilterConfig;

	uint8_t targetChannel = 0;

	int omega_sign = 1;

	uint8_t button = 0;
	uint8_t p_button = 0;


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
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  //MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */


  ntshell_usr_init(&nts);

  MotorTest_Init(&motorTest);

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x400 << 5;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x70c << 5;
	sFilterConfig.FilterMaskIdLow = 0x0006;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if(HAL_CAN_ConfigFilter(&hcan1,&sFilterConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
	{
	  Error_Handler();
	}

  // CAN Start
  if(HAL_CAN_Start(&hcan1) != HAL_OK)
  {
	  Error_Handler();
  }

  // ADC Start
  if(HAL_ADC_Start_IT(&hadc1) != HAL_OK)
  {
	  Error_Handler();
  }



  printf("Send request to main.\n");
  requestMotorParam();

  HAL_Delay(500);

  // Timer Start
  if(HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
  {
	  Error_Handler();
  }


  //printf("Hello\n");

  HAL_Delay(1000);


  int count = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  ntshell_execute(&nts);



	  HAL_Delay(10);


	  targetChannel = getChannel();

	  LED_blink_times = targetChannel;

	  button = !HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin);

	  if(button && !p_button)
	  {
		  omega_sign *= -1;
	  }
	  p_button = button;

	  float Iq_ref[4] = {0.0, 0.0, 0.0, 0.0};

	  Iq_ref[targetChannel] = omega_sign * getVolume() * 15.0f;

	  driveMotor_speed(Iq_ref);

	  printf("%Iq_ref = %f\nIq_int = %6d, theta_int = %6d, omega_int = %6d\n", Iq_ref[targetChannel], Iq_res_int16, theta_res_uint16, omega_res_int16);

	  printf("\e[2A");

	  //printf_cdc("ID:%d, speed_ref=%.1f\n", targetChannel, Iq_ref[targetChannel]);


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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 288;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 10;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIM = 10;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLSAIP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 18;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_LED_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USER_BUTTON_Pin CH_1_Pin CH_2_Pin CH_3_Pin 
                           CH_4_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin|CH_1_Pin|CH_2_Pin|CH_3_Pin 
                          |CH_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
