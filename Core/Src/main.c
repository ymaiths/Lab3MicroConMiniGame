/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "ModBusRTU.h"

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
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t SPIRx[10];
uint8_t SPITx[10];
uint8_t Button[4];
uint8_t mode ;
uint8_t LightOrder[10];
uint8_t PushOrder[10];
uint8_t AnsCorrect=1;
uint8_t num;
uint8_t a[10];
uint8_t Rand;
uint8_t Equal;
uint8_t RandNum;
uint8_t LedOff;
uint8_t StartGame;
uint8_t Score;
uint8_t FinalScore;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void IODIRA_Init();
void IODIRB_Init();
void SPITxRx_Setup();
void SPITxRx_readIO();
void SPITxRx_writeIO();
uint8_t AnsCheck();
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
  MX_SPI3_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  SPITxRx_Setup();
  IODIRB_Init(); //Led
  IODIRA_Init(); //Button
  num = 4;
  mode = 1;
  StartGame = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	Rand = Rand +1;
	if(StartGame == 1){
		static uint64_t timestamp = 0;
		if (HAL_GetTick() > timestamp){
			if(mode == 1){
				  RandNum = (Rand%4)+1;
				  SPITxRx_writeIO();
				  a[0] += 1;
				  timestamp = HAL_GetTick() + 800;

			}else if(mode ==2){
				SPITxRx_readIO();

			}else if(mode ==3)
				a[9]=AnsCheck();
		}
	}else if(StartGame == 0){
		SPITxRx_readIO();
		}
	Score = num-4;


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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 169;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1145;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim16, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LPUART1_TX_Pin LPUART1_RX_Pin */
  GPIO_InitStruct.Pin = LPUART1_TX_Pin|LPUART1_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF12_LPUART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void SPITxRx_readIO() //mode2
{
	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2)){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0); // CS Select
		SPITx[0] = 0b01000001;
		SPITx[1] = 0x12;
		SPITx[2] = 0;

		if(SPIRx[2]== 14){
			a[4] = 1;
			StartGame =1;
		}else if(SPIRx[2]== 13){
			a[4] = 2;
		}else if(SPIRx[2]== 11){
			a[4] = 3;
		}else if(SPIRx[2]== 7){
			a[4] = 4;
		}else{
			a[4] = 5;
		}

		if(a[5]!=a[4] && a[4] == 5 && a[5] != 0){
			PushOrder[a[2]-1] = a[5];
			a[2]=a[2]+1;
		}
		// set a[5] = old a[4]
		a[5] = a[4];
		a[8] +=1 ;

		if(a[2] > num){
			a[7] =1;
			mode = 3;
			a[2] = 1;
		}


	HAL_SPI_TransmitReceive_IT(&hspi3, SPITx, SPIRx, 3);

}}

void SPITxRx_writeIO(){ //mode1
	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2)){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0); // CS Select
		if(LedOff == 0){
			SPITx[0] = 0b01000000;
			SPITx[1] = 0x15;
				if(RandNum == 1){
					SPITx[2] = ~(0b00000001);
					LedOff = 1 ;
					a[3] = 1;
				}else if(RandNum == 2){
					SPITx[2] = ~(0b00000010);
					LedOff = 1 ;
					a[3] = 2;
				}else if(RandNum == 3){
					SPITx[2] = ~(0b00000100);
					LedOff = 1 ;
					a[3] = 3;
				}else if(RandNum == 4){
					SPITx[2] = ~(0b00001000);
					LedOff = 1 ;
					a[3] = 4;
				}
			LightOrder[a[1]] = a[3];
			a[1] = a[1]+1;

		}else if(a[1] >= num){
			LedOff = 0;
			SPITx[2] = 0b00000000;
			mode = 2;
			a[1] = 0;


		}
		else if(LedOff == 1){
			SPITx[0] = 0b01000000;
			SPITx[1] = 0x15;
			SPITx[2] = 0b11111111;
			LedOff = 0;
		}
		HAL_SPI_TransmitReceive_IT(&hspi3, SPITx, SPIRx, 3);


	}}

uint8_t AnsCheck(){
	if(LightOrder[a[6]]!=PushOrder[a[6]]){
		StartGame = 0;
		mode = 1;
		FinalScore = Score;
		num = 4;
		a[2] = 1;
	}else{
		a[6] = a[6] + 1;
	}

	if(a[6] > num){
		a[6] = 0;
		num += 1;
		mode = 1;
	}
}




//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//	if()
//}
void SPITxRx_Setup(){
	//CS pulse
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0); // CS Select
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1); // CS deSelect
	HAL_Delay(1);
}

void IODIRB_Init(){
	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2)){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0); // CS Select
		SPITx[0] = 0b01000000;
		SPITx[1] = 0x01;
		SPITx[2] = 0;
		HAL_SPI_TransmitReceive_IT(&hspi3, SPITx, SPIRx, 3);
}}

void IODIRA_Init(){
	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2)){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0); // CS Select
		SPITx[0] = 0b01000001;
		SPITx[1] = 0x00;
		SPITx[2] = 0;
		HAL_SPI_TransmitReceive_IT(&hspi3, SPITx, SPIRx, 3);
}}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1); //CS dnSelect
}



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
