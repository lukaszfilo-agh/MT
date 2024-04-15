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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
volatile uint8_t callback_counter = 0;
volatile uint8_t thousands;
volatile uint8_t hundreds;
volatile uint8_t tens;
volatile uint8_t ones;
volatile uint16_t number_disp = 0;

// Serial variables
volatile uint8_t buf_tx[100];
volatile uint16_t tx_cnt;
volatile uint16_t ADC_READ[2];
volatile uint8_t flag_tx = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void com_change(uint8_t com);
void disp_change(uint8_t num);
void num_show(void);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	if (hadc->Instance == ADC1) {
		if (flag_tx == 0) {
			tx_cnt = snprintf(buf_tx, sizeof(buf_tx), "%d, %d\n", ADC_READ[0], ADC_READ[1]);
			HAL_UART_Transmit_DMA(&huart2, buf_tx, tx_cnt);
			flag_tx = 1;
		}

	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM11) {
		// Green LED FLASH
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		num_show();

	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		flag_tx = 0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
	}
}

void num_show(void) {
	// Displaying numbers
	if(callback_counter == 0) {
		// decoding
		ones = number_disp % 10;
		tens = (number_disp / 10) % 10;
		hundreds = (number_disp / 100) % 10;
		thousands = (number_disp / 1000) % 10;

		// turning off all displays
		com_change(0);

		// number set
		disp_change(thousands);

		// turning on display 1
		if (thousands != 0) {
		com_change(1);
		}
		callback_counter++;
	}
	else if (callback_counter == 1) {
		// turning off all displays
		com_change(0);

		// number set
		disp_change(hundreds);

		// turning on display 2
		if (hundreds != 0 || thousands != 0) {
		com_change(2);
		}
		callback_counter++;
	}
	else if (callback_counter == 2) {
		// turning off all displays
		com_change(0);

		// number set
		disp_change(tens);

		// turning on display 3
		if (thousands != 0 || hundreds != 0 || tens != 0) {
		com_change(3);
		}
		callback_counter++;
	}
	else if (callback_counter == 3) {
		// turning off all displays
		com_change(0);

		// number set
		disp_change(ones);

		// turning on display 4
		com_change(4);
		callback_counter = 0;
	}
}

void com_change(uint8_t com) {
	switch(com) {
	case 1:
		HAL_GPIO_WritePin(GPIOC, COM1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, COM2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM4_Pin, GPIO_PIN_SET);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOC, COM1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, COM3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM4_Pin, GPIO_PIN_SET);
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOC, COM1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, COM4_Pin, GPIO_PIN_SET);
		break;
	case 4:
		HAL_GPIO_WritePin(GPIOC, COM1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM4_Pin, GPIO_PIN_RESET);
		break;
	default:
		HAL_GPIO_WritePin(GPIOC, COM1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM4_Pin, GPIO_PIN_SET);
		break;
	}
}

void disp_change(uint8_t num) {
	switch(num) {
	case 0:
		HAL_GPIO_WritePin(GPIOC, A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, C_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, D_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, E_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, F_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, G_Pin, GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOC, A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, C_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, D_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, E_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, F_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, G_Pin, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOC, A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, C_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, D_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, E_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, F_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, G_Pin, GPIO_PIN_SET);
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOC, A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, C_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, D_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, E_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, F_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, G_Pin, GPIO_PIN_SET);
		break;
	case 4:
		HAL_GPIO_WritePin(GPIOC, A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, C_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, D_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, E_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, F_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, G_Pin, GPIO_PIN_SET);
		break;
	case 5:
		HAL_GPIO_WritePin(GPIOC, A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, C_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, D_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, E_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, F_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, G_Pin, GPIO_PIN_SET);
		break;
	case 6:
		HAL_GPIO_WritePin(GPIOC, A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, C_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, D_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, E_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, F_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, G_Pin, GPIO_PIN_SET);
		break;
	case 7:
		HAL_GPIO_WritePin(GPIOC, A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, C_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, D_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, E_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, F_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, G_Pin, GPIO_PIN_RESET);
		break;
	case 8:
		HAL_GPIO_WritePin(GPIOC, A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, C_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, D_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, E_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, F_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, G_Pin, GPIO_PIN_SET);
		break;
	case 9:
		HAL_GPIO_WritePin(GPIOC, A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, C_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, D_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, E_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, F_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, G_Pin, GPIO_PIN_SET);
		break;
	default:
		HAL_GPIO_WritePin(GPIOC, A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, C_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, D_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, E_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, F_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, G_Pin, GPIO_PIN_RESET);
		break;
	}
}

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM11_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // timers start with interruption
  HAL_TIM_Base_Start_IT(&htim11);

  // DMA ADC & temp
  HAL_TIM_Base_Start(&htim2);
  HAL_ADC_Start_DMA(&hadc1, ADC_READ, 2);

  // Serial code
  HAL_UART_Transmit_DMA(&huart2, buf_tx, 2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  number_disp = ADC_READ[0];
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 8399;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 9;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, G_Pin|D_Pin|E_Pin|C_Pin
                          |B_Pin|F_Pin|A_Pin|DP_Pin
                          |COM4_Pin|COM3_Pin|COM2_Pin|COM1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : G_Pin D_Pin E_Pin C_Pin
                           B_Pin F_Pin A_Pin DP_Pin
                           COM4_Pin COM3_Pin COM2_Pin COM1_Pin */
  GPIO_InitStruct.Pin = G_Pin|D_Pin|E_Pin|C_Pin
                          |B_Pin|F_Pin|A_Pin|DP_Pin
                          |COM4_Pin|COM3_Pin|COM2_Pin|COM1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
