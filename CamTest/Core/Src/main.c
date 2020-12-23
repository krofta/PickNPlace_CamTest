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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define TIMER_FREQ 45.0
/*
#define T0H 0.35
#define T0L 0.8
#define T1H 0.7
#define T1L 0.6
*/

#define T0H 3.0
#define T0L 7.0
#define T1H 7.0
#define T1L 3.0
#define TRESET 50

uint16_t cnt;
uint8_t led_data[36];
uint16_t led_pos;
uint8_t led_mask;
uint8_t led_lastbit;
uint16_t low_CCR1, low_ARR, high_CCR1, high_ARR, treset_ARR;
long double period;

void write_ws2812();

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DCMI_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void DCMICompleteCallback(DCMI_HandleTypeDef *hdcmi){
	frame_flag = 1;
}
void DCMIErrorCallback(DCMI_HandleTypeDef *hdcmi){
	// just do random stuff for debug
	int i = 0;
	i++;
}


void write_ws2812(){
	led_pos = 0;
	led_lastbit = 0;
	led_mask = 0b10000000;
	cnt = 0;

	if(led_data[0] & led_mask){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)high_CCR1);
		__HAL_TIM_SET_AUTORELOAD(&htim3, (uint32_t)high_ARR);
		//TIM2->CCR1 = (uint32_t)high_CCR1;
		//TIM2->ARR = (uint32_t)high_ARR;
	}else{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)low_CCR1);
		__HAL_TIM_SET_AUTORELOAD(&htim3, (uint32_t)low_ARR);
		//TIM2->CCR1 = (uint32_t)low_CCR1;
		//TIM2->ARR = (uint32_t)low_ARR;
	}

	//TIM2->CCER |= TIM_CCER_CC1E;	//enable pwm channel to pin
	//TIM2->CR1 |= TIM_CR1_CEN;		// enable channel 1
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
}

void show_neopixels(){
	led_pos = 0; //set the interupt to start at first byte
	led_lastbit = 0;
	led_mask = 0B10000000; //set the interupt to start at second bit

	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
	//TIM2->SR &= ~TIM_SR_UIF; // clear UIF flag
	//TIM2->DIER |= TIM_DIER_UIE; //enable interupt flag to be generated to start transmission
}

void Neopixel_setup(void){

	//calculate all the timings.
	period = 1 / TIMER_FREQ;
	low_CCR1 = round(T0H / period);
	low_ARR = round((T0H + T0L) / period);
	high_CCR1 = round(T1H / period);
	high_ARR = round((T1H + T1L) / period);
	treset_ARR = ceil(TRESET / period);

	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //enable port A clock
	//GPIOA->MODER |= GPIO_MODER_MODER15_1; //setup pin 12 on port d to AF mode
	//GPIOA->AFR[1] = (GPIOA->AFR[1] & (0b1111<<(4*(12-8))) | 0b0010<<(4*(12-8))); //setup pin 12 on port D to AF timer 2-5


	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //enable the timer4 clock
	TIM2->PSC = 0;   //set prescale to zero as timer has to go as fast as posible
	TIM2->CCMR1 = (TIM2->CCMR1 & ~(0b110<<4)) | (0b110<<4); //set PWM mode 110
	TIM2->CCR1 = 0; //set to zero so that the pin stay low until transmission
	TIM2->ARR = treset_ARR; //set to timing for reset LEDs
	TIM2->CCER |= TIM_CCER_CC1E; //enable output to pin.
	TIM2->CR1 |= TIM_CR1_CEN; //Disable channel 1. This bit is used to start and stop transmission.
	TIM2->CR1 |= TIM_CR1_ARPE; //buffer ARR
	TIM2->CCMR1 |= TIM_CCMR1_OC1PE; //buffer CCR1
	TIM2->DIER &= ~TIM_DIER_UIE; // ensure we are not enabling interrupt flag to be generated this bit is used to start/stop transmission
	TIM2->CR1 |= TIM_CR1_CEN; //enable channel 1.

	NVIC_EnableIRQ(TIM2_IRQn); // Enable interrupt(NVIC level)
}
/*
void TIM2_IRQHandler(void){

	TIM2->SR &= ~TIM_SR_UIF; // clear UIF flag

		if(led_pos<sizeof(led_data)){
			if(led_data[led_pos] & led_mask){
				TIM2->CCR1 = high_CCR1;
				TIM2->ARR = high_ARR;
			}else{
				TIM2->CCR1 = low_CCR1;
				TIM2->ARR = low_ARR;
			}
			if(led_mask==1){
				led_mask = 0B10000000;
				led_pos+=1;
			}else led_mask = led_mask >> 1;
		}else{
			TIM2->CCR1 = 0; //set to zero so that pin stays low
			TIM2->ARR = treset_ARR; //set to timing for reset LEDs
			TIM2->DIER &= ~TIM_DIER_UIE; //disable interrupt flag to end transmission.
		}
}
*/




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_DCMI_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //OV7670_init(&hi2c2);
  //memset(frame_buffer, 0, sizeof(uint16_t) *IMG_ROWS * IMG_COLUMNS);
  for(unsigned int i = 0; i < IMG_ROWS * IMG_COLUMNS; i++){
	  frame_buffer[i] = 0;
  }

  ov7670_init(&hdcmi, &hdma_dcmi, &hi2c2);
  ov7670_config(1);




  //HAL_DMA_RegisterCallback(&hdma_dcmi, HAL_DMA_XFER_CPLT_CB_ID, &DCMICompleteCallback);
  //HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, &frame_buffer, IMG_ROWS * IMG_COLUMNS/2);

  //Neopixel_setup();

  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*
	  HAL_StatusTypeDef status = HAL_DMA_PollForTransfer(&hdma_dcmi, HAL_DMA_FULL_TRANSFER, 10000);
	  if(status == HAL_OK){
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, &frame_buffer, IMG_ROWS * IMG_COLUMNS/2);
	  }
		*/


	  for (uint8_t i = 0; i < 36; i++)
		  led_data[i] = 0;  //use low values so that it does blind the camera
	  //HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
	  write_ws2812();
	  HAL_Delay(1000);
	  for (uint8_t i = 0; i < 36; i++)
		  led_data[i] = 0b00000010;  //use low values so that it does blind the camera
	  //HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
	  write_ws2812();

	  HAL_Delay(1000);

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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C2;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */
  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */
  //hdcmi.DMA_Handle->

  //hdcmi.XferSize = IMG_ROWS * IMG_COLUMNS;
  //hdcmi.pBuffPtr = frame_buffer;

  //hdcmi.DMA_Handle->XferCpltCallback = DCMICompleteCallback;
  //hdcmi.DMA_Handle->XferErrorCallback = DCMIErrorCallback;

  /* USER CODE END DCMI_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x203073FF;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 200;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim3){
		//TIM2->SR &= ~TIM_SR_UIF; // clear UIF flag
		if(led_pos<sizeof(led_data)){
			if(led_data[led_pos] & led_mask){
				//TIM2->CCR1 = high_CCR1;
				//TIM2->ARR = high_ARR;
				__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint32_t)high_CCR1);
				__HAL_TIM_SET_AUTORELOAD(htim, (uint32_t)high_ARR);
			}else{
				__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint32_t)low_CCR1);
				__HAL_TIM_SET_AUTORELOAD(htim, (uint32_t)low_ARR);
				//TIM2->CCR1 = low_CCR1;
				//TIM2->ARR = low_ARR;
			}
			if(led_mask==1){
				led_mask = 0b10000000;
				led_pos++;
			}else led_mask = led_mask >> 1;
		}else{
			//HAL_TIM_Base_Stop_IT(htim);
			//TIM3->CCR1 = 0; //set to zero so that pin stays low
			//TIM3->ARR = treset_ARR; //set to timing for reset LEDs
			//TIM3->DIER &= ~TIM_DIER_UIE; //disable interrupt flag to end transmission.
			HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
		}
	}

}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){

}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){

}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
