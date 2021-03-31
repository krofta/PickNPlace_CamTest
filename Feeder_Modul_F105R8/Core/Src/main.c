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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbh_hid.h"
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
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

char barcode[100];
char* act_char= &barcode[0];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void USBH_HID_EventCallback(USBH_HandleTypeDef *phost){
	if(USBH_HID_GetDeviceType(phost) == HID_KEYBOARD){
		HID_KEYBD_Info_TypeDef *keyboard_info;
		keyboard_info = USBH_HID_GetKeybdInfo(phost);
		char key = USBH_HID_GetASCIICode(keyboard_info);
		if(key == '\0')
			return;
		if(key == '\r' || key == '\n'){
			*act_char = '\0';
			act_char = &barcode[0];

		}
		else{
			*act_char++ = key;
		}

		if(act_char > &barcode[99])
			act_char = &barcode[0];

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
	uint8_t sprocket_started = 0;
	uint8_t tape_started = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  //MX_GPIO_Init();
  //HAL_GPIO_WritePin(LED_Error_GPIO_Port, LED_Error_Pin, GPIO_PIN_SET);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USB_HOST_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED_Error_GPIO_Port, LED_Error_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_Status_GPIO_Port, LED_Status_Pin, GPIO_PIN_RESET);
  HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    //HAL_GPIO_WritePin(PWM_A), GPIO_Pin, PinState)

    // TEST SPROCKET
    if(HAL_GPIO_ReadPin(BTN_TBACK_GPIO_Port, BTN_TBACK_Pin) == GPIO_PIN_RESET && sprocket_started == 0){
    	HAL_Delay(150);
    	if(HAL_GPIO_ReadPin(BTN_TBACK_GPIO_Port, BTN_TBACK_Pin) == GPIO_PIN_RESET){
    		sprocket_started = 1;
    		//HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(DIR_A_GPIO_Port, DIR_A_Pin, GPIO_PIN_SET);
    	}
    }
    if(HAL_GPIO_ReadPin(BTN_TBACK_GPIO_Port, BTN_TBACK_Pin) == GPIO_PIN_SET && sprocket_started == 1){
    	HAL_Delay(150);
    	if(HAL_GPIO_ReadPin(BTN_TBACK_GPIO_Port, BTN_TBACK_Pin) == GPIO_PIN_SET){
    		sprocket_started = 0;
    		//HAL_TIMEx_PWMN_Stop(&htim3, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(DIR_A_GPIO_Port, DIR_A_Pin, GPIO_PIN_RESET);
    	}
    }
    if(HAL_GPIO_ReadPin(BTN_TFOR_GPIO_Port, BTN_TFOR_Pin) == GPIO_PIN_RESET && sprocket_started == 0){
    	HAL_Delay(150);
    	if(HAL_GPIO_ReadPin(BTN_TFOR_GPIO_Port, BTN_TFOR_Pin) == GPIO_PIN_RESET){
    		sprocket_started = 2;
    		//HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_1);
			//HAL_GPIO_WritePin(DIR_A_GPIO_Port, DIR_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(DIR_A2_GPIO_Port, DIR_A2_Pin, GPIO_PIN_SET);
    	}
    }
    if(HAL_GPIO_ReadPin(BTN_TFOR_GPIO_Port, BTN_TFOR_Pin) == GPIO_PIN_SET && sprocket_started == 2){
    	HAL_Delay(150);
    	if(HAL_GPIO_ReadPin(BTN_TFOR_GPIO_Port, BTN_TFOR_Pin) == GPIO_PIN_SET){
    		sprocket_started = 0;
    		//HAL_TIMEx_PWMN_Stop(&htim3, TIM_CHANNEL_1);
			//HAL_GPIO_WritePin(DIR_A_GPIO_Port, DIR_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(DIR_A2_GPIO_Port, DIR_A2_Pin, GPIO_PIN_RESET);
    	}
    }


    // TAPE MOTOR TEST
    if(HAL_GPIO_ReadPin(BTN_SBACK_GPIO_Port, BTN_SBACK_Pin) == GPIO_PIN_RESET && tape_started == 0){
    	HAL_Delay(150);
    	if(HAL_GPIO_ReadPin(BTN_SBACK_GPIO_Port, BTN_SBACK_Pin) == GPIO_PIN_RESET){
    		tape_started = 1;
    		//HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET);
    	}
    }
    if(HAL_GPIO_ReadPin(BTN_SBACK_GPIO_Port, BTN_SBACK_Pin) == GPIO_PIN_SET && tape_started == 1){
    	HAL_Delay(150);
    	if(HAL_GPIO_ReadPin(BTN_SBACK_GPIO_Port, BTN_SBACK_Pin) == GPIO_PIN_SET){
    		tape_started = 0;
    		//HAL_TIMEx_PWMN_Stop(&htim3, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_RESET);
    	}
    }
    if(HAL_GPIO_ReadPin(BTN_SFOR_GPIO_Port, BTN_SFOR_Pin) == GPIO_PIN_RESET && tape_started == 0){
    	HAL_Delay(150);
    	if(HAL_GPIO_ReadPin(BTN_SFOR_GPIO_Port, BTN_SFOR_Pin) == GPIO_PIN_RESET){
    		tape_started = 2;
    		//HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_1);
			//HAL_GPIO_WritePin(DIR_A_GPIO_Port, DIR_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(DIR_B2_GPIO_Port, DIR_B2_Pin, GPIO_PIN_SET);
    	}
    }
    if(HAL_GPIO_ReadPin(BTN_SFOR_GPIO_Port, BTN_SFOR_Pin) == GPIO_PIN_SET && tape_started == 2){
    	HAL_Delay(150);
    	if(HAL_GPIO_ReadPin(BTN_SFOR_GPIO_Port, BTN_SFOR_Pin) == GPIO_PIN_SET){
    		tape_started = 0;
    		//HAL_TIMEx_PWMN_Stop(&htim3, TIM_CHANNEL_1);
			//HAL_GPIO_WritePin(DIR_A_GPIO_Port, DIR_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(DIR_B2_GPIO_Port, DIR_B2_Pin, GPIO_PIN_RESET);
    	}
    }

    /*HAL_GPIO_WritePin(LED_Error_GPIO_Port, LED_Error_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_Status_GPIO_Port, LED_Status_Pin, GPIO_PIN_RESET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(LED_Error_GPIO_Port, LED_Error_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_Status_GPIO_Port, LED_Status_Pin, GPIO_PIN_SET);
    HAL_Delay(500);*/


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL10;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
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
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim1.Init.Prescaler = 48000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 500;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR_A_Pin|DIR_A2_Pin|DIR_B2_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DIR_B_Pin|MODE_Pin|LED_Error_Pin|LED_Status_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BTN_TFOR_Pin BTN_TBACK_Pin TAPE_MICROBTN_Pin */
  GPIO_InitStruct.Pin = BTN_TFOR_Pin|BTN_TBACK_Pin|TAPE_MICROBTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_A_Pin DIR_A2_Pin DIR_B2_Pin PA10 */
  GPIO_InitStruct.Pin = DIR_A_Pin|DIR_A2_Pin|DIR_B2_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_B_Pin MODE_Pin LED_Error_Pin LED_Status_Pin */
  GPIO_InitStruct.Pin = DIR_B_Pin|MODE_Pin|LED_Error_Pin|LED_Status_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ADDR7_Pin ADDR6_Pin ADDR5_Pin ADDR4_Pin
                           BTN_SFOR_Pin BTN_SBACK_Pin */
  GPIO_InitStruct.Pin = ADDR7_Pin|ADDR6_Pin|ADDR5_Pin|ADDR4_Pin
                          |BTN_SFOR_Pin|BTN_SBACK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ADDR3_Pin ADDR2_Pin ADDR1_Pin ADDR0_Pin */
  GPIO_InitStruct.Pin = ADDR3_Pin|ADDR2_Pin|ADDR1_Pin|ADDR0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
