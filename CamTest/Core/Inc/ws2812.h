/*
 * ws2812.h
 *
 *  Created on: September 21, 2020
 *      Author: Dmitriy Semenov
 *		Github: https://github.com/Crazy-Geeks/STM32-WS2812B-DMA
 *		License: MIT
 *		(c) #Crazy_Geeks
 */

#ifndef SRC_WS2812_H_
#define SRC_WS2812_H_

//--------------------------------------------------
/*INCLUDES*/
#include "stm32f7xx_hal.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
//--------------------------------------------------
/*USR SETTINGS*/
#define LED_COUNT 29						// Count of led in your strip (LED_COUNT%12 = 0)
#define BRIGHT 255 							// 0-255
#define TIM_HANDLE	htim3					// use "htimX", where X is number of ur timer
#define TIM_CH	TIM_CHANNEL_2				// use "TIM_CHANNEL_X", where X is number of ur channel
//--------------------------------------------------
/*SYS DEFINES*/
#define DELAY_LEN 50						// Delay period
#define ARRAY_LEN DELAY_LEN + LED_COUNT*24	// Length of DMA Buffer
//#define HIGH 65								// HIGH Byte period
//#define LOW 26								// LOW Byte period
#define TLOW 0.35
#define THIGH 0.90
#define HIGH (uint32_t)((THIGH/(1/TIMER_FREQ))-1.0)								// HIGH Byte period
#define LOW (uint32_t)((TLOW/(1/TIMER_FREQ))-1.0)								// LOW Byte period
//#define HIGH 80
//#define LOW 30
//#define PWM_PERIOD 1.15
#define PWM_PERIOD 1.4
#define TIMER_FREQ 108.0					// Timer Frequenz in 10^9
#define PWM_ARR (uint32_t)((PWM_PERIOD/(1/TIMER_FREQ))-1.0)

extern TIM_HandleTypeDef TIM_HANDLE;		// Initalization of timer handler
//--------------------------------------------------

//--------------------------------------------------
#define BitIsSet(reg, bit) ((reg & (1<<bit)) != 0)	// Search Bit in integer
//--------------------------------------------------

void led_init(void);		// Initalization
void set_led(uint8_t Rpixel , uint8_t Gpixel, uint8_t Bpixel, uint16_t posX);  // Draw pixel in X position by RGB
void led_fill (uint8_t Rpix, uint8_t Gpix, uint8_t Bpix);	// Fill strip with color
void led_clear(void);	// Fill strip with black
void led_show(void);	// Recieve DMA Buffer to strip
#endif /* SRC_WS2812_H_ */
