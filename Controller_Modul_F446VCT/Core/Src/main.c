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
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"
#include "string.h"
#include "file_handling.h"
#include "st7789.h"

//#include "../tmc/ic/TMC2226/TMC2226_Register.h"
//#include "../tmc/helpers/CRC.h"

#include "tmc2226_wrapper.h"
#include "../tmc/ic/TMC2226/TMC2226.h"
#define CRC8_GEN 0x07

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern BoardTypeDef motorBoard;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

char com_buf[64];
uint8_t com_bytes_available = 0;

uint32_t cursorLine = 0;
uint8_t cursorChanged = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char *data = "Moin von Nikos Pick n Place Maschine\r\n";



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
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_ADC1_Init();
  MX_TIM14_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char cmd = 0;
  memset(&com_buf,0,sizeof(com_buf));
  // Timer for status led
  HAL_TIM_Base_Start_IT(&htim14);

  mount_sd();

  //init SPI display ST7789 320x240 px
  ST7789_Init();
  // fill displaywith color
  ST7789_Fill_Color(DARKBLUE);
  // Backlight of spi display to 50%
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);

  // encoder channel start
  HAL_TIM_Encoder_Start_IT(&htim8, TIM_CHANNEL_ALL);

  //TMC2226_init();

  //motorBoard.ch[0].enableDriver(0, DRIVER_DISABLE);
  //HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState)


  //motorBoard.ch[0].moveBy(0, 1000);
	ConfigurationTypeDef config;
	config.reset = reset;
	config.restore = restore;
	TMC2226TypeDef TMC2226;
	//PinsTypeDef Pins;
	IOPinTypeDef  ENN;
	IOPinTypeDef  STEP;
	IOPinTypeDef  DIR;
	IOPinTypeDef  DIAG;
	IOPinTypeDef  INDEX;

	ENN.GPIOx   		= EN_ROT_GPIO_Port;
	ENN.GPIO_Pin		= EN_ROT_Pin;
	STEP.GPIOx    	= STEP_ROT_GPIO_Port;
	STEP.GPIO_Pin		= STEP_ROT_Pin;
	DIR.GPIOx			= DIR_ROT_GPIO_Port;
	DIR.GPIO_Pin		= DIR_ROT_Pin;
	DIAG.GPIOx    	= DIAG_ROT_GPIO_Port;
	DIAG.GPIO_Pin		= DIAG_ROT_Pin;
	INDEX.GPIOx   	= INDEX_ROT_GPIO_Port;
	INDEX.GPIO_Pin	= INDEX_ROT_Pin;
	tmc_fillCRC8Table(0x07, true, 1);

	tmc2226_init(&TMC2226, 0, 0, &config, &tmc2226_defaultRegisterResetState[0]);
	StepDir_init(STEPDIR_PRECISION);
	StepDir_setPins(0, &STEP, &DIR, &DIAG);
	StepDir_setVelocityMax(0, 40000);
	StepDir_setAcceleration(0, 20000);

	// Timer val 320?!
	HAL_TIM_Base_Start_IT(&htim2);



	writeConfiguration(&TMC2226);


	uint32_t val = 2700 * 50 / VREF_FULLSCALE;
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1, val);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2, val);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3, val);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3, val);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	//setVREF(0,500); // mV
	//enableDriver(0, DRIVER_ENABLE);
	int position = 0;

/*
	moveTo(1, 10000);
	moveTo(2, 10000);
	moveTo(3, 10000);
	*/
  while (1)
  {


	  //ST7789_Test();
	  // test virtual com port
	  if(cursorChanged){
		  cursorChanged = 0;
		  char buf[10];
		  sprintf(buf,"%3u", cursorLine);
		  ST7789_WriteString(10, 20, buf, Font_11x18, RED, WHITE);
		  moveTo(0, cursorLine * 100);

	  }

	  if(com_bytes_available){
		  com_bytes_available = 0;
		  /*
		  if(!strcmp(com_buf,"moin")){
			  CDC_Transmit_FS((uint8_t*)data, strlen(data));
		  }
		  */
		  if(com_buf[0] == 'R' && com_buf[1] == ' ' ){
			  int32_t rot  = atoi(&com_buf[2]);
			  moveTo(0, rot);

		  }


  		  //Get_string(buffer);
  		  //int len = cmdlength(buffer);
		  int len = strlen(com_buf);
  		  get_path();

  		  if (!(strncmp ("ls", com_buf,len))) cmd = 'l';
  		  if (!(strncmp ("mkdir", com_buf,len))) cmd = 'm';
  		  if (!(strncmp ("mkfil", com_buf, len))) cmd = 'c';
  		  if (!(strncmp ("read", com_buf, len))) cmd = 'r';
  		  if (!(strncmp ("write", com_buf, len))) cmd = 'w';
  		  if (!(strncmp ("rm", com_buf, len))) cmd = 'd';
  		  if (!(strncmp ("update", com_buf, len))) cmd = 'u';
  		  if (!(strncmp ("checkfile", com_buf, len))) cmd = 'f';
  		  if (!(strncmp ("checksd", com_buf, len))) cmd = 's';

  		  switch (cmd)
  		  {
  		  	  case ('l'):
				scan_files(path);
				cmd =0;
				clear_buffer();
				clear_path();
				break;
  			  case ('m'):
				CDC_Transmit_FS("enter dir name\n", strlen("enter dir name\n"));
				while (!com_bytes_available);
				memcpy(&path,&com_buf, sizeof(path));
				com_bytes_available = 0;
				create_dir (path);
				cmd=0;
				clear_path();
				break;
  			  case ('c'):
				CDC_Transmit_FS("enter file name\n", strlen("enter file name\n"));
				while (!com_bytes_available);
				memcpy(&path,&com_buf, sizeof(path));
				com_bytes_available = 0;
				create_file(path);
				cmd = 0;
				clear_path();
				break;
  			  case ('r'):
				CDC_Transmit_FS("enter file name\n", strlen("enter file name\n"));
				while (!com_bytes_available);
				memcpy(&path,&com_buf, sizeof(path));
				com_bytes_available = 0;
				read_file (path);
				cmd = 0;
				clear_path();
				break;
  			  case ('d'):
				remove_file(path);
				cmd = 0;
				clear_path();
				break;
  			  case ('w'):
				CDC_Transmit_FS("enter file name\n", strlen("enter file name\n"));
				while (!com_bytes_available);
				memcpy(&path,&com_buf, sizeof(path));
				com_bytes_available = 0;
				write_file (path);
				cmd = 0;
				clear_path();
				break;
  			  case ('u'):
				update_file (path);
				cmd = 0;
				clear_path();
				break;
  			  case ('f'):
				check_file(path);
				cmd = 0;
				clear_path();
				break;
			  case ('s'):
				check_sd();
				cmd = 0;
				clear_path();
				break;
			  default :
				clear_buffer();
				clear_path();
				break;
  		  }

	  }

	  //HAL_Delay(1);
	  //HAL_GPIO_TogglePin(STEP_ROT_GPIO_Port, STEP_ROT_Pin);


	  // OUTPUTS test:outputs funktionieren
	  HAL_GPIO_WritePin(VENTIL1_GPIO_Port, VENTIL1_Pin, 1);
	  HAL_GPIO_WritePin(VENTIL2_GPIO_Port, VENTIL2_Pin, 1);
	  HAL_GPIO_WritePin(VACUUM_PUMP_GPIO_Port, VACUUM_PUMP_Pin, 1);
	  HAL_GPIO_WritePin(OUTPUT_RES1_GPIO_Port, OUTPUT_RES1_Pin, 1);
	  HAL_GPIO_WritePin(OUTPUT_RES2_GPIO_Port, OUTPUT_RES2_Pin, 1);
	  HAL_GPIO_WritePin(OUTPUT_RES3_GPIO_Port, OUTPUT_RES3_Pin, 1);
	  HAL_GPIO_WritePin(OUTPUT_RES4_GPIO_Port, OUTPUT_RES4_Pin, 1);
	  HAL_GPIO_WritePin(OUTPUT_RES5_GPIO_Port, OUTPUT_RES5_Pin, 1);
	  HAL_GPIO_WritePin(OUTPUT_RES6_GPIO_Port, OUTPUT_RES6_Pin, 1);
	  HAL_GPIO_WritePin(OUTPUT_RES7_GPIO_Port, OUTPUT_RES7_Pin, 1);

	  // TEST inputs: inputs funktionieren
	  int input = 0;
	  if(!HAL_GPIO_ReadPin(ENDSTOP_X1_GPIO_Port, ENDSTOP_X1_Pin))
		  input++;
	  if(!HAL_GPIO_ReadPin(ENDSTOP_X2_GPIO_Port, ENDSTOP_X2_Pin))
		  input++;
	  if(!HAL_GPIO_ReadPin(ENDSTOP_Y1_GPIO_Port, ENDSTOP_Y1_Pin))
		  input++;
	  if(!HAL_GPIO_ReadPin(ENDSTOP_Y2_GPIO_Port, ENDSTOP_Y2_Pin))
		  input++;
	  if(!HAL_GPIO_ReadPin(ENDSTOP_Z1_GPIO_Port, ENDSTOP_Z1_Pin))
		  input++;
	  if(!HAL_GPIO_ReadPin(ENDSTOP_Z2_GPIO_Port, ENDSTOP_Z2_Pin))
		  input++;
	  if(!HAL_GPIO_ReadPin(ENDSTOP_ROT1_GPIO_Port, ENDSTOP_ROT1_Pin))
		  input++;
	  if(!HAL_GPIO_ReadPin(ENDSTOP_ROT2_GPIO_Port, ENDSTOP_ROT2_Pin))
		  input++;
	  if(!HAL_GPIO_ReadPin(INPUT_RES1_GPIO_Port, INPUT_RES1_Pin))
		  input++;
	  if(!HAL_GPIO_ReadPin(INPUT_RES2_GPIO_Port, INPUT_RES2_Pin))
		  input++;
	  //HAL_GPIO_WritePin(LED_Status_GPIO_Port, LED_Status_Pin, input > 0);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDIO|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
  PeriphClkInitStruct.SdioClockSelection = RCC_SDIOCLKSOURCE_CLK48;
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
  sConfig.Channel = ADC_CHANNEL_1;
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
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 4;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

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
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 42;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 25;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 100;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 7;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 7;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 8400;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 20;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 8400;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 2000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, EN_Z_Pin|DIR_Z_Pin|STEP_Z_Pin|EN_X_Pin
                          |DIR_X_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, EN_ROT_Pin|DIR_ROT_Pin|SPI_CS_Pin|SPI_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STEP_Y_Pin|SPI_RESET_Pin|LED_Status_Pin|OUTPUT_RES6_Pin
                          |OUTPUT_RES5_Pin|OUTPUT_RES4_Pin|STEP_X_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUTPUT_RES7_Pin|OUTPUT_RES3_Pin|OUTPUT_RES2_Pin|STEP_ROT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OUTPUT_RES1_Pin|VACUUM_PUMP_Pin|VENTIL2_Pin|VENTIL1_Pin
                          |EN_Y_Pin|DIR_Y_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EN_Z_Pin DIR_Z_Pin STEP_Z_Pin EN_X_Pin
                           DIR_X_Pin */
  GPIO_InitStruct.Pin = EN_Z_Pin|DIR_Z_Pin|STEP_Z_Pin|EN_X_Pin
                          |DIR_X_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : DIAG_Z_Pin INDEX_Z_Pin ENDSTOP_X1_Pin SDIO_CD_Pin
                           DIAG_X_Pin INDEX_X_Pin ENDSTOP_Z2_Pin ENDSTOP_Z1_Pin */
  GPIO_InitStruct.Pin = DIAG_Z_Pin|INDEX_Z_Pin|ENDSTOP_X1_Pin|SDIO_CD_Pin
                          |DIAG_X_Pin|INDEX_X_Pin|ENDSTOP_Z2_Pin|ENDSTOP_Z1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : ENDSTOP_X2_Pin ENDSTOP_Y1_Pin ENDSTOP_Y2_Pin DIAG_ROT_Pin
                           INDEX_ROT_Pin ENC_BTN_Pin INPUT_RES2_Pin */
  GPIO_InitStruct.Pin = ENDSTOP_X2_Pin|ENDSTOP_Y1_Pin|ENDSTOP_Y2_Pin|DIAG_ROT_Pin
                          |INDEX_ROT_Pin|ENC_BTN_Pin|INPUT_RES2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_ROT_Pin DIR_ROT_Pin SPI_CS_Pin SPI_DC_Pin */
  GPIO_InitStruct.Pin = EN_ROT_Pin|DIR_ROT_Pin|SPI_CS_Pin|SPI_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : STEP_Y_Pin SPI_RESET_Pin LED_Status_Pin OUTPUT_RES6_Pin
                           OUTPUT_RES5_Pin OUTPUT_RES4_Pin STEP_X_Pin */
  GPIO_InitStruct.Pin = STEP_Y_Pin|SPI_RESET_Pin|LED_Status_Pin|OUTPUT_RES6_Pin
                          |OUTPUT_RES5_Pin|OUTPUT_RES4_Pin|STEP_X_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OUTPUT_RES7_Pin OUTPUT_RES3_Pin OUTPUT_RES2_Pin STEP_ROT_Pin */
  GPIO_InitStruct.Pin = OUTPUT_RES7_Pin|OUTPUT_RES3_Pin|OUTPUT_RES2_Pin|STEP_ROT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : OUTPUT_RES1_Pin VACUUM_PUMP_Pin VENTIL2_Pin VENTIL1_Pin
                           EN_Y_Pin DIR_Y_Pin */
  GPIO_InitStruct.Pin = OUTPUT_RES1_Pin|VACUUM_PUMP_Pin|VENTIL2_Pin|VENTIL1_Pin
                          |EN_Y_Pin|DIR_Y_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : DIAG_Y_Pin INDEX_Y_Pin INPUT_RES1_Pin */
  GPIO_InitStruct.Pin = DIAG_Y_Pin|INDEX_Y_Pin|INPUT_RES1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ENDSTOP_ROT2_Pin ENDSTOP_ROT1_Pin */
  GPIO_InitStruct.Pin = ENDSTOP_ROT2_Pin|ENDSTOP_ROT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
