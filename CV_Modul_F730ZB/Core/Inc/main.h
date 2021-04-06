/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

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
#define OV7670_PWD_Pin GPIO_PIN_2
#define OV7670_PWD_GPIO_Port GPIOE
#define OV7670_RET_Pin GPIO_PIN_3
#define OV7670_RET_GPIO_Port GPIOE
#define LED_Status_Pin GPIO_PIN_13
#define LED_Status_GPIO_Port GPIOC
#define LED_Error_Pin GPIO_PIN_14
#define LED_Error_GPIO_Port GPIOC
#define ADDR0_Pin GPIO_PIN_12
#define ADDR0_GPIO_Port GPIOB
#define ADDR1_Pin GPIO_PIN_11
#define ADDR1_GPIO_Port GPIOD
#define ADDR2_Pin GPIO_PIN_12
#define ADDR2_GPIO_Port GPIOD
#define ADDR3_Pin GPIO_PIN_13
#define ADDR3_GPIO_Port GPIOD
#define ST7789_RST_Pin GPIO_PIN_10
#define ST7789_RST_GPIO_Port GPIOG
#define ST7789_DC_Pin GPIO_PIN_13
#define ST7789_DC_GPIO_Port GPIOG
#define ST7789_CS_Pin GPIO_PIN_14
#define ST7789_CS_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
