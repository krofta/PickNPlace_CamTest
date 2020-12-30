/*
 * ov7670.h
 *
 *  Created on: 2017/08/25
 *      Author: take-iwiw
 */

#ifndef OV7670_OV7670_H_
#define OV7670_OV7670_H_
#include "common.h"
#include "main.h"
#define OV7670_PWD_PORT OV7670_PWD_GPIO_Port
#define OV767_PWD_Pin OV7670_PWD_Pin



#define OV7670_MODE_QVGA_RGB565 0
#define OV7670_MODE_QVGA_YUV    1

#define OV7670_CAP_CONTINUOUS   0
#define OV7670_CAP_SINGLE_FRAME 1

RET ov7670_init(DCMI_HandleTypeDef *p_hdcmi, DMA_HandleTypeDef *p_hdma_dcmi, I2C_HandleTypeDef *p_hi2c);
RET ov7670_config(uint32_t mode);
RET ov7670_startCap(uint32_t capMode, uint32_t destAddress);
RET ov7670_stopCap();
void ov7670_registerCallback(void (*cbHsync)(uint32_t h), void (*cbVsync)(uint32_t v));
#define ov7670_powerdown() HAL_GPIO_WritePin(OV7670_PWD_PORT, OV767_PWD_Pin, GPIO_PIN_RESET)
#define ov7670_powerup() HAL_GPIO_WritePin(OV7670_PWD_PORT, OV767_PWD_Pin, GPIO_PIN_SET)

#endif /* OV7670_OV7670_H_ */
