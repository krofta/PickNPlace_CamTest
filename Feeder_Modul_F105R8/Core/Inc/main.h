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
#include "stm32f1xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BTN_TFOR_Pin GPIO_PIN_1
#define BTN_TFOR_GPIO_Port GPIOA
#define BTN_TBACK_Pin GPIO_PIN_2
#define BTN_TBACK_GPIO_Port GPIOA
#define DIR_A_Pin GPIO_PIN_5
#define DIR_A_GPIO_Port GPIOA
#define DIR_A2_Pin GPIO_PIN_6
#define DIR_A2_GPIO_Port GPIOA
#define DIR_B2_Pin GPIO_PIN_7
#define DIR_B2_GPIO_Port GPIOA
#define DIR_B_Pin GPIO_PIN_4
#define DIR_B_GPIO_Port GPIOC
#define MODE_Pin GPIO_PIN_5
#define MODE_GPIO_Port GPIOC
#define ADDR7_Pin GPIO_PIN_12
#define ADDR7_GPIO_Port GPIOB
#define ADDR6_Pin GPIO_PIN_13
#define ADDR6_GPIO_Port GPIOB
#define ADDR5_Pin GPIO_PIN_14
#define ADDR5_GPIO_Port GPIOB
#define ADDR4_Pin GPIO_PIN_15
#define ADDR4_GPIO_Port GPIOB
#define ADDR3_Pin GPIO_PIN_6
#define ADDR3_GPIO_Port GPIOC
#define ADDR2_Pin GPIO_PIN_7
#define ADDR2_GPIO_Port GPIOC
#define ADDR1_Pin GPIO_PIN_8
#define ADDR1_GPIO_Port GPIOC
#define ADDR0_Pin GPIO_PIN_9
#define ADDR0_GPIO_Port GPIOC
#define TAPE_MICROBTN_Pin GPIO_PIN_15
#define TAPE_MICROBTN_GPIO_Port GPIOA
#define LED_Error_Pin GPIO_PIN_10
#define LED_Error_GPIO_Port GPIOC
#define LED_Status_Pin GPIO_PIN_11
#define LED_Status_GPIO_Port GPIOC
#define BTN_SFOR_Pin GPIO_PIN_3
#define BTN_SFOR_GPIO_Port GPIOB
#define BTN_SBACK_Pin GPIO_PIN_4
#define BTN_SBACK_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
