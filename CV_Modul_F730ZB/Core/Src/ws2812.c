/*
 * ws2812.c
 *
 *  Created on: September 21, 2020
 *      Author: Dmitriy Semenov
 *		Github: https://github.com/Crazy-Geeks/STM32-WS2812B-DMA
 *		License: MIT
  *		(c) #Crazy_Geeks
 */

#include "ws2812.h"  // include header file
//----------------------------------------------------------------------------
// DMA Buffer


void led_init(void) {	// DMA Buffer initalization (fill strip to black)
	//*BUF_DMA = (__IO uint16_t*) (SDRAM_BANK_ADDR + WRITE_READ_ADDR + (320*240 * 2));
	int i;
	for (i = DELAY_LEN; i < ARRAY_LEN; i++)
		BUF_DMA[i] = LOW;
}
//------------------------------------------------------------------
void set_led(uint8_t Rpixel, uint8_t Gpixel, uint8_t Bpixel, uint16_t posX) {
	// Convert value to brightness
	Rpixel = (Rpixel * BRIGHT) / 255;
	Gpixel = (Gpixel * BRIGHT) / 255;
	Bpixel = (Bpixel * BRIGHT) / 255;

	// Bit operations
	volatile uint16_t i;
	for (i = 0; i < 8; i++) {
		if (BitIsSet(Rpixel,(7-i)) == 1) {
			BUF_DMA[DELAY_LEN + posX * 24 + i + 8] = HIGH;
		} else {
			BUF_DMA[DELAY_LEN + posX * 24 + i + 8] = LOW;
		}
		if (BitIsSet(Gpixel,(7-i)) == 1) {
			BUF_DMA[DELAY_LEN + posX * 24 + i + 0] = HIGH;
		} else {
			BUF_DMA[DELAY_LEN + posX * 24 + i + 0] = LOW;
		}
		if (BitIsSet(Bpixel,(7-i)) == 1) {
			BUF_DMA[DELAY_LEN + posX * 24 + i + 16] = HIGH;
		} else {
			BUF_DMA[DELAY_LEN + posX * 24 + i + 16] = LOW;
		}
	}
}

// Fill led operation
void led_fill(uint8_t Rpix, uint8_t Gpix, uint8_t Bpix) {
	for (uint16_t i = 0; i < LED_COUNT; i++) {
		set_led(Rpix, Gpix, Bpix, i);
	}
}

// Clear led operation
void led_clear(void) {
	for (uint16_t i = 0; i < LED_COUNT; i++) {
		set_led(0, 0, 0, i);
	}

}

// Function to recieve DMA buffer to timer PWM
void led_show(void) {
	//uint16_t *Ptr_Dest = (uint16_t *)&BUF_DMA;
	//__DSB();
	SCB_CleanDCache_by_Addr((uint32_t*)(((uint32_t)BUF_DMA) & ~(uint32_t)0x1F), ARRAY_LEN+32);

	HAL_TIM_PWM_Start_DMA(&TIM_HANDLE, TIM_CH, (uint32_t*) &BUF_DMA,ARRAY_LEN);
	//HAL_TIM_PWM_Start_DMA(&TIM_HANDLE, TIM_CH, (uint32_t*) Ptr_Dest,ARRAY_LEN);
}

//------------------------------------------------------------------
