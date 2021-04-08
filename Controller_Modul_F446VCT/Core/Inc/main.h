/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EN_Z_Pin GPIO_PIN_2
#define EN_Z_GPIO_Port GPIOE
#define DIAG_Z_Pin GPIO_PIN_3
#define DIAG_Z_GPIO_Port GPIOE
#define INDEX_Z_Pin GPIO_PIN_4
#define INDEX_Z_GPIO_Port GPIOE
#define DIR_Z_Pin GPIO_PIN_5
#define DIR_Z_GPIO_Port GPIOE
#define ENDSTOP_X1_Pin GPIO_PIN_6
#define ENDSTOP_X1_GPIO_Port GPIOE
#define ENDSTOP_X2_Pin GPIO_PIN_13
#define ENDSTOP_X2_GPIO_Port GPIOC
#define ENDSTOP_Y1_Pin GPIO_PIN_14
#define ENDSTOP_Y1_GPIO_Port GPIOC
#define ENDSTOP_Y2_Pin GPIO_PIN_15
#define ENDSTOP_Y2_GPIO_Port GPIOC
#define EN_ROT_Pin GPIO_PIN_0
#define EN_ROT_GPIO_Port GPIOC
#define DIAG_ROT_Pin GPIO_PIN_1
#define DIAG_ROT_GPIO_Port GPIOC
#define INDEX_ROT_Pin GPIO_PIN_2
#define INDEX_ROT_GPIO_Port GPIOC
#define DIR_ROT_Pin GPIO_PIN_3
#define DIR_ROT_GPIO_Port GPIOC
#define STEP_Y_Pin GPIO_PIN_0
#define STEP_Y_GPIO_Port GPIOA
#define SPI_RESET_Pin GPIO_PIN_4
#define SPI_RESET_GPIO_Port GPIOA
#define LED_Status_Pin GPIO_PIN_7
#define LED_Status_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_4
#define SPI_CS_GPIO_Port GPIOC
#define SPI_DC_Pin GPIO_PIN_5
#define SPI_DC_GPIO_Port GPIOC
#define SDIO_CD_Pin GPIO_PIN_7
#define SDIO_CD_GPIO_Port GPIOE
#define STEP_Z_Pin GPIO_PIN_9
#define STEP_Z_GPIO_Port GPIOE
#define EN_X_Pin GPIO_PIN_10
#define EN_X_GPIO_Port GPIOE
#define DIAG_X_Pin GPIO_PIN_11
#define DIAG_X_GPIO_Port GPIOE
#define INDEX_X_Pin GPIO_PIN_12
#define INDEX_X_GPIO_Port GPIOE
#define DIR_X_Pin GPIO_PIN_13
#define DIR_X_GPIO_Port GPIOE
#define OUTPUT_RES7_Pin GPIO_PIN_10
#define OUTPUT_RES7_GPIO_Port GPIOB
#define OUTPUT_RES3_Pin GPIO_PIN_14
#define OUTPUT_RES3_GPIO_Port GPIOB
#define OUTPUT_RES2_Pin GPIO_PIN_15
#define OUTPUT_RES2_GPIO_Port GPIOB
#define OUTPUT_RES1_Pin GPIO_PIN_8
#define OUTPUT_RES1_GPIO_Port GPIOD
#define VACUUM_PUMP_Pin GPIO_PIN_9
#define VACUUM_PUMP_GPIO_Port GPIOD
#define VENTIL2_Pin GPIO_PIN_10
#define VENTIL2_GPIO_Port GPIOD
#define VENTIL1_Pin GPIO_PIN_11
#define VENTIL1_GPIO_Port GPIOD
#define ENC_BTN_Pin GPIO_PIN_9
#define ENC_BTN_GPIO_Port GPIOC
#define OUTPUT_RES6_Pin GPIO_PIN_8
#define OUTPUT_RES6_GPIO_Port GPIOA
#define OUTPUT_RES5_Pin GPIO_PIN_9
#define OUTPUT_RES5_GPIO_Port GPIOA
#define OUTPUT_RES4_Pin GPIO_PIN_10
#define OUTPUT_RES4_GPIO_Port GPIOA
#define STEP_X_Pin GPIO_PIN_15
#define STEP_X_GPIO_Port GPIOA
#define INPUT_RES2_Pin GPIO_PIN_12
#define INPUT_RES2_GPIO_Port GPIOC
#define EN_Y_Pin GPIO_PIN_3
#define EN_Y_GPIO_Port GPIOD
#define DIAG_Y_Pin GPIO_PIN_4
#define DIAG_Y_GPIO_Port GPIOD
#define INDEX_Y_Pin GPIO_PIN_5
#define INDEX_Y_GPIO_Port GPIOD
#define DIR_Y_Pin GPIO_PIN_6
#define DIR_Y_GPIO_Port GPIOD
#define INPUT_RES1_Pin GPIO_PIN_7
#define INPUT_RES1_GPIO_Port GPIOD
#define STEP_ROT_Pin GPIO_PIN_4
#define STEP_ROT_GPIO_Port GPIOB
#define ENDSTOP_ROT2_Pin GPIO_PIN_8
#define ENDSTOP_ROT2_GPIO_Port GPIOB
#define ENDSTOP_ROT1_Pin GPIO_PIN_9
#define ENDSTOP_ROT1_GPIO_Port GPIOB
#define ENDSTOP_Z2_Pin GPIO_PIN_0
#define ENDSTOP_Z2_GPIO_Port GPIOE
#define ENDSTOP_Z1_Pin GPIO_PIN_1
#define ENDSTOP_Z1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
