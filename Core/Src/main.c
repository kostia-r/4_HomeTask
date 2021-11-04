/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEMPERATURE_DELAY 5000 //temperature sensor polling time, ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile uint32_t temperature_timer = 0; //see stm32f4xx_it.c line 45,187
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void ToogleLED_and_UARTTx(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, UART_HandleTypeDef *huart, uint8_t *pData_on, uint8_t *pData_off, uint8_t size_Data_on,
		uint8_t size_Data_off)
{
	HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
	if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET)
	{
		HAL_UART_Transmit(huart, (uint8_t*) pData_on, size_Data_on, 10);
	}
	else
	{
		HAL_UART_Transmit(huart, (uint8_t*) pData_off, size_Data_off, 10);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
	case SWT1_Pin:
	{
		ToogleLED_and_UARTTx(GPIOD, LED_BLUE_Pin, &huart3, (uint8_t*) "LED Blue ON\r\n", (uint8_t*) "LED Blue OFF\r\n", 11 + 2, 12 + 2);
	}
		break;
	case SWT3_Pin:
	{
		ToogleLED_and_UARTTx(GPIOD, LED_ORANGE_Pin, &huart3, (uint8_t*) "LED Orange ON\r\n", (uint8_t*) "LED Orange OFF\r\n", 13 + 2, 14 + 2);
	}
		break;
	case SWT4_Pin:
	{
		ToogleLED_and_UARTTx(GPIOD, LED_RED_Pin, &huart3, (uint8_t*) "LED Red ON\r\n", (uint8_t*) "LED Red OFF\r\n", 10 + 2, 11 + 2);
	}
		break;
	case SWT5_Pin:
	{
		ToogleLED_and_UARTTx(GPIOD, LED_GREEN_Pin, &huart3, (uint8_t*) "LED Green ON\r\n", (uint8_t*) "LED Green OFF\r\n", 12 + 2, 13 + 2);
	}
		break;
	case SWT2_Pin:
	{
		HAL_UART_Transmit(&huart3, (uint8_t*) "Invalid key!\r\n", 12 + 2, 10);
	}
		break;
	}
}

void UARTRx(void)
{
	uint8_t rcvBuf[1];
	HAL_StatusTypeDef result;
	result = HAL_UART_Receive(&huart3, rcvBuf, 1, 10);
	if (result == HAL_OK)
	{
		switch (rcvBuf[0])
		{
		case '2': //analogue of the SWT1 button on the board
		{
			ToogleLED_and_UARTTx(GPIOD, LED_BLUE_Pin, &huart3, (uint8_t*) "LED Blue ON\r\n", (uint8_t*) "LED Blue OFF\r\n", 11 + 2, 12 + 2);
		}
			break;

		case '8': //analogue of the SWT3 button on the board
		{
			ToogleLED_and_UARTTx(GPIOD, LED_ORANGE_Pin, &huart3, (uint8_t*) "LED Orange ON\r\n", (uint8_t*) "LED Orange OFF\r\n", 13 + 2, 14 + 2);
		}
			break;
		case '6': //analogue of the SWT4 button on the board
		{
			ToogleLED_and_UARTTx(GPIOD, LED_RED_Pin, &huart3, (uint8_t*) "LED Red ON\r\n", (uint8_t*) "LED Red OFF\r\n", 10 + 2, 11 + 2);
		}
			break;
		case '4': //analogue of the SWT5 button on the board
		{
			ToogleLED_and_UARTTx(GPIOD, LED_GREEN_Pin, &huart3, (uint8_t*) "LED Green ON\r\n", (uint8_t*) "LED Green OFF\r\n", 12 + 2, 13 + 2);
		}
			break;
		default: //analogue of the SWT2 button on the board
			HAL_UART_Transmit(&huart3, (uint8_t*) "Invalid key!\r\n", 12 + 2, 10);
			break;
		}
	}
}

uint32_t MAP_Invert(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
	return out_max - (x - in_min) * (out_max - out_min) / (in_max - in_min);
}

void GetExternalTemperature(void)
{
	volatile HAL_StatusTypeDef adcPoolResult;
	int32_t valueExTemp = 0;
	int16_t externalTemp = 0;
	double voltageExTemp = 0;
	HAL_ADC_Start(&hadc1);
	adcPoolResult = HAL_ADC_PollForConversion(&hadc1, 100);
	if (adcPoolResult == HAL_OK)
	{
		valueExTemp = HAL_ADC_GetValue(&hadc1); // read value from external temperature sensor
		voltageExTemp = (double) valueExTemp * (3.3 / 4096) * 100; //convert valueExTemp to voltage (volts x100)
		externalTemp = (int16_t) MAP_Invert(voltageExTemp, 2, 202, 1, 100); //convert voltage to temperature x10 in the range from 1 to 100°C. 2 - voltage x100 at a temperature of 100°C, 202  - voltage x100 at a temperature of 0°C
		HAL_UART_Transmit(&huart3, (uint8_t*) "External temperature: ", 22, 10);
		if (externalTemp < 10)
		{
			char temperatureStr[1];
			temperatureStr[0] = '0' + externalTemp;
			HAL_UART_Transmit(&huart3, (uint8_t*) temperatureStr, 4, 10);
		}
		else if (externalTemp >= 10)
		{
			char temperatureStr[2];
			temperatureStr[0] = '0' + externalTemp / 10;
			temperatureStr[1] = '0' + externalTemp % 10;
			HAL_UART_Transmit(&huart3, (uint8_t*) temperatureStr, 4, 10);
		}
		else
		{
			HAL_UART_Transmit(&huart3, (uint8_t*) "Out of range of temperature measurement!", 41, 10);
		}
		HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
	}
	HAL_ADC_Stop(&hadc1);
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
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		temperature_timer = 0; //see stm32f4xx_it.c line 45,187
		while (temperature_timer < TEMPERATURE_DELAY)
		{
			UARTRx(); //receiving data from UART
		}
		GetExternalTemperature();
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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

	ADC_ChannelConfTypeDef sConfig =
	{ 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
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
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin | LED_ORANGE_Pin | LED_RED_Pin | LED_BLUE_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : LED_GREEN_Pin LED_ORANGE_Pin LED_RED_Pin LED_BLUE_Pin */
	GPIO_InitStruct.Pin = LED_GREEN_Pin | LED_ORANGE_Pin | LED_RED_Pin | LED_BLUE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : SWT4_Pin SWT5_Pin SWT3_Pin SWT1_Pin */
	GPIO_InitStruct.Pin = SWT4_Pin | SWT5_Pin | SWT3_Pin | SWT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : SWT2_Pin */
	GPIO_InitStruct.Pin = SWT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SWT2_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
