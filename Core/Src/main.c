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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "notesFrequencies.h"
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
unsigned char input = '0';
TIM_HandleTypeDef *buzzerPwmTimer;
uint32_t buzzerPwmChannel;
int duration = 0;
int play = 0;
int freq = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

void buzzer_init();
void buzzer_change_tone(uint16_t freq, uint16_t volume);
void decode(int num, int BCD[4]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USART3_UART_Init();
	MX_TIM3_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	buzzer_init();
//  buzzer_change_tone(500, 10);
	HAL_UART_Receive_IT(&huart3, &input, sizeof(input));
	HAL_TIM_Base_Start_IT(&htim3);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 47;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 999;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 9600;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
			GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_8 | GPIO_PIN_9
					| GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

	/*Configure GPIO pins : PA0 PA1 PA2 PA8
	 PA9 PA10 PA11 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_8
			| GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB12 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PB15 */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	static int state = 0;
//	GPIOA;
	static uint16_t IC[4] = { GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11 };
	static int bcd[4] = { 0 };
	if (htim->Instance == TIM3) {
		if (play > 0) {
			buzzer_change_tone(freq, 5);
			play--;
		} else {
			buzzer_change_tone(0, 0);
		}

		if (state == 0) {
			state++;
			decode(duration / 100, bcd);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			for (int i = 0; i < 4; i++) {
				HAL_GPIO_WritePin(GPIOA, IC[i], bcd[i]);
			}
		} else if (state == 1) {
			state++;
			decode((duration / 10) % 10, bcd);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
			for (int i = 0; i < 4; i++) {
				HAL_GPIO_WritePin(GPIOA, IC[i], bcd[i]);
			}
		} else {
			state = 0;
			decode(duration % 10, bcd);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			for (int i = 0; i < 4; i++) {
				HAL_GPIO_WritePin(GPIOA, IC[i], bcd[i]);
			}
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		HAL_UART_Receive_IT(&huart3, &input, sizeof(input));
		HAL_UART_Transmit(&huart3, &input, sizeof(input), 1000);
		switch (input) {
		case 'a':
			play = duration;
			freq = NOTE_B0;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
			break;
		case 's':
			play = duration;
			freq = NOTE_C1;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
			break;
		case 'd':
			play = duration;
			freq = NOTE_CS1;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
			break;
		case 'f':
			play = duration;
			freq = NOTE_A1;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
			break;
		case 'g':
			play = duration;
			freq = NOTE_A2;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
			break;
		case 'h':
			play = duration;
			freq = NOTE_A3;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
			break;
		case 'j':
			play = duration;
			freq = NOTE_A4;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
			break;
		case 'k':
			play = duration;
			freq = NOTE_A5;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
			break;
		case 'l':
			play = duration;
			freq = NOTE_A6;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
			break;
		case 'q':
			play = duration;
			freq = NOTE_C7;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
			break;
		case 'w':
			play = duration;
			freq = NOTE_C6;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
			break;
		default:
			break;
		}
	}
}

void buzzer_init() {
	buzzerPwmTimer = &htim2;
	buzzerPwmChannel = TIM_CHANNEL_4;
	HAL_TIM_PWM_Start(buzzerPwmTimer, buzzerPwmChannel);
}

void buzzer_change_tone(uint16_t freq, uint16_t volume) {
	if (freq == 0 || freq > 20000) {
		__HAL_TIM_SET_COMPARE(buzzerPwmTimer, buzzerPwmChannel, 0);
	} else {
		const uint32_t internalClockFreq = HAL_RCC_GetSysClockFreq();
		const uint32_t prescaler = 1 + internalClockFreq / freq / 60000;
		const uint32_t timerClock = internalClockFreq / prescaler;
		const uint32_t periodCycles = timerClock / freq;
		const uint32_t pulseWidth = volume * periodCycles / 1000 / 2;

		buzzerPwmTimer->Instance->PSC = prescaler - 1;
		buzzerPwmTimer->Instance->ARR = periodCycles - 1;
		buzzerPwmTimer->Instance->EGR = TIM_EGR_UG;

		__HAL_TIM_SET_COMPARE(buzzerPwmTimer, buzzerPwmChannel, pulseWidth);
	}
}

void decode(int num, int BCD[4]) {
	switch (num) {
	case 0:
		BCD[3] = 0;
		BCD[2] = 0;
		BCD[1] = 0;
		BCD[0] = 0;
		break;
	case 1:
		BCD[3] = 0;
		BCD[2] = 0;
		BCD[1] = 0;
		BCD[0] = 1;
		break;
	case 2:
		BCD[3] = 0;
		BCD[2] = 0;
		BCD[1] = 1;
		BCD[0] = 0;
		break;
	case 3:
		BCD[3] = 0;
		BCD[2] = 0;
		BCD[1] = 1;
		BCD[0] = 1;
		break;
	case 4:
		BCD[3] = 0;
		BCD[2] = 1;
		BCD[1] = 0;
		BCD[0] = 0;
		break;
	case 5:
		BCD[3] = 0;
		BCD[2] = 1;
		BCD[1] = 0;
		BCD[0] = 1;
		break;
	case 6:
		BCD[3] = 0;
		BCD[2] = 1;
		BCD[1] = 1;
		BCD[0] = 0;
		break;
	case 7:
		BCD[3] = 0;
		BCD[2] = 1;
		BCD[1] = 1;
		BCD[0] = 1;
		break;
	case 8:
		BCD[3] = 1;
		BCD[2] = 0;
		BCD[1] = 0;
		BCD[0] = 0;
		break;
	case 9:
		BCD[3] = 1;
		BCD[2] = 0;
		BCD[1] = 0;
		BCD[0] = 1;
		break;
	default:
		break;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	static uint32_t current = 0;
	if (GPIO_Pin == GPIO_PIN_15) {
		if (HAL_GetTick() - current > 200) {
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
			duration += 100;
			duration %= 1100;
			buzzer_change_tone(duration, 10);
			current = HAL_GetTick();
		}
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
