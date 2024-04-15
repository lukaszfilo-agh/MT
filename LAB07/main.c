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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t callback_counter = 0;
volatile uint8_t thousands;
volatile uint8_t hundreds;
volatile uint8_t tens;
volatile uint8_t ones;
volatile uint16_t number = 1234;

uint8_t buf_tx[100];
volatile uint16_t tx_cnt;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void com_change(uint8_t com);
void disp_change(uint8_t num);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM11) {
		// Green LED FLASH
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		// Displaying numbers
		if(callback_counter == 0) {
			// decoding
			ones = number % 10;
			tens = (number / 10) % 10;
			hundreds = (number / 100) % 10;
			thousands = (number / 1000) % 10;

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
  MX_USART2_UART_Init();
  MX_TIM11_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  // timer with interruption
  HAL_TIM_Base_Start_IT(&htim11);

  // config mems via spi
  uint8_t MEMSTxBuf[2];
  uint8_t MEMSRxBuf[2];
  MEMSTxBuf[0] = 0x00 | 0x20;
  MEMSTxBuf[1] = 0x47;

  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, MEMSTxBuf, 2, 10);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(250);
	  uint8_t MEMS[3];

	  // x axis
	  MEMSRxBuf[0] = 0x80 | 0x29;
	  MEMSRxBuf[1] = 0x29;

	  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_Receive(&hspi1, MEMSRxBuf, 2, 10);
	  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
	  MEMS[0] = MEMSRxBuf[1];

	  // y axis
	  MEMSRxBuf[0] = 0x80 | 0x2B;
	  MEMSRxBuf[1] = 0x29;

	  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_Receive(&hspi1, MEMSRxBuf, 2, 10);
	  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
	  MEMS[1] = MEMSRxBuf[1];

	  // z axis
	  MEMSRxBuf[0] = 0x80 | 0x2D;
	  MEMSRxBuf[1] = 0x29;

	  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_Receive(&hspi1, MEMSRxBuf, 2, 10);
	  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
	  MEMS[2] = MEMSRxBuf[1];

	  tx_cnt = snprintf(buf_tx, sizeof(buf_tx), "x = %d, y = %d, z = %d\n", MEMS[0], MEMS[1], MEMS[2]);

	  HAL_UART_Transmit(&huart2, buf_tx, tx_cnt, 100);
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin|LD2_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : SPI1_NSS_Pin LD2_Pin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
