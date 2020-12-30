/*
 * menu.c
 *
 *  Created on: Dec 30, 2020
 *      Author: niko
 */

#include "menu.h"
#include "st7789.h"
#include "globals.h"
#include "ov7670.h"
uint16_t actual_menu = ACT_MENU_MAIN;
uint16_t cursorLine = 0;

// extern variables
extern TIM_HandleTypeDef htim8;
extern uint8_t btn_enc;

char *preprocessing_menu_opts[OPTS_PREPROCESSING] = {
		"Histogramm (normal)",
		"Histogramm (kumulativ)",
		"Grauwert-Dehnung",
		"Histogramm-Ebnung",
		"Mittelwert-Filter",
		"Median-Filter",
		"Gauss-Filter",
		"Back"
};

char *image_processing_menu_opts[OPTS_IMAGE_PROCESSING] = {
		"Capture image",
		"Show image",
		"Binary processing",
		"Preprocessing",
		"Edge detection",
		"Texture detection",
		"Segmentation",
		"Back"
};
char *main_menu_opts[OPTS_MAIN_MENU] = {
		"Image processing",
		"Light settings",
		"Camera settings"
};

void capture_image(int manu){
	led_fill(55,55,55);
	led_show();

	HAL_Delay(10);

	while(1){
		if(pic_captured == 0 /*&& pic_written == 1*/){
			ov7670_startCap(OV7670_CAP_SINGLE_FRAME, &framebuffer);
			pic_written = 0;
			pic_captured = 2;
		}
		if(pic_captured == 1){
			pic_captured = 0;
			ov7670_stopCap();
			//ST7789_WriteDataDMA(framebuffer, sizeof(framebuffer),0,0,320,240);
			ST7789_DrawImage(0,0,320,240,framebuffer);
			break;
		}

	}
	led_fill(0,0,0);
	led_show();
	for(int x = 0; x < MAXXDIM ; x++){
		for(int y = 0; y < MAXYDIM; y++){
			int tmp = x*y;
			img[x][y] = (((framebuffer[tmp]& (0xF8<<8))>>8) +
					(((framebuffer[tmp]& (0x07<<8))>>3) + ((framebuffer[tmp]& (0xF8))>>3)) +
					((framebuffer[tmp]& (0x1F)))) / 3;
		}
	}

	if(manu){
		ST7789_WriteString(10, 0, "Press to continue", Font_11x18, WHITE, BLACK);
		while(!btn_enc);
		HAL_Delay(400);
		btn_enc = 0;
	}
	return;
}

void setCursor(int pos){
	ST7789_DrawRectangle(0, pos * MENU_LINE_HEIGHT, MENU_LINE_WIDTH - 1,
			pos * MENU_LINE_HEIGHT + MENU_LINE_HEIGHT, WHITE);
}
void delCursor(int pos){
	ST7789_DrawRectangle(0, pos * MENU_LINE_HEIGHT, MENU_LINE_WIDTH - 1,
			pos * MENU_LINE_HEIGHT + MENU_LINE_HEIGHT, BLACK);
}
void print_menu(char *options[], int opt_count)
{
	ST7789_Fill_Color(BLACK);
	for(int i = 0; i < opt_count; i++){
		ST7789_WriteString(10, i*MENU_LINE_HEIGHT, options[i], Font_11x18, WHITE, BLACK);
	}
}
void menu_pre_processing(){
	uint16_t last_cursorline;
	//HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	// set global variable, that actual manu is main menu
	actual_menu = ACT_MENU_PREPROCESSING;
	// set encoder count to zero
	htim8.Instance->CNT = 0;
	last_cursorline = htim8.Instance->CNT;
	// set max timer val for encoder
	__HAL_TIM_SetAutoreload(&htim8,OPTS_PREPROCESSING-1);
	print_menu(preprocessing_menu_opts, OPTS_PREPROCESSING);
	setCursor(cursorLine);
	while (1)
	{
		if(cursorLine != last_cursorline){
			delCursor(last_cursorline);
			setCursor(cursorLine);
			last_cursorline = cursorLine;
		}
		if(btn_enc){
			HAL_Delay(300);
			btn_enc = 0;
			switch (cursorLine)
			{
			case 0:

				break;
			case 1:
				// TODO: call function
				break;
			case 2:
				// TODO: call function
				break;
			case 3:
				// TODO: call function
				break;
			case 4:
				// TODO: call function
				break;
			case 5:
				// TODO: call function
				break;
			case 6:
				// TODO: call function
				break;
			case 7:
				return;
			default:break;
			}
			// when back in this menu -> call print menu function
			htim8.Instance->CNT = cursorLine = last_cursorline = 0;
			__HAL_TIM_SetAutoreload(&htim8,OPTS_IMAGE_PROCESSING-1);
			print_menu(image_processing_menu_opts, OPTS_IMAGE_PROCESSING);
			setCursor(cursorLine);
		}
	}
}

void menu_image_processing()
{
	uint16_t last_cursorline;
	//HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	// set global variable, that actual manu is main menu
	actual_menu = ACT_MENU_IMAGE_PROCESSING;
	// set encoder count to zero
	htim8.Instance->CNT = 0;
	last_cursorline = htim8.Instance->CNT;
	// set max timer val for encoder
	__HAL_TIM_SetAutoreload(&htim8,OPTS_IMAGE_PROCESSING-1);
	print_menu(image_processing_menu_opts, OPTS_IMAGE_PROCESSING);
	setCursor(cursorLine);
	while (1)
	{
		if(cursorLine != last_cursorline){
			delCursor(last_cursorline);
			setCursor(cursorLine);
			last_cursorline = cursorLine;
		}
		if(btn_enc){
			HAL_Delay(300);
			//HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
			btn_enc = 0;
			switch (cursorLine)
			{
			case 0:
				capture_image(1);
				// TODO: call function
				break;
			case 1:
				// TODO: call function
				break;
			case 2:
				// TODO: call function
				break;
			case 3:
				menu_pre_processing();
				break;
			case 4:
				// TODO: call function
				break;
			case 5:
				// TODO: call function
				break;
			case 6:
				// TODO: call function
				break;
			case 7:
				return;
			default:break;
			}
			// when back in this menu -> call print menu function
			htim8.Instance->CNT = cursorLine = last_cursorline = 0;
			__HAL_TIM_SetAutoreload(&htim8,OPTS_IMAGE_PROCESSING-1);
			print_menu(image_processing_menu_opts, OPTS_IMAGE_PROCESSING);
			setCursor(cursorLine);
		}
	}
}

void menu()
{
	uint16_t last_cursorline;
	//HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	// set global variable, that actual manu is main menu
	actual_menu = ACT_MENU_MAIN;
	// set encoder count to zero
	htim8.Instance->CNT = 0;
	last_cursorline = htim8.Instance->CNT;
	// set max timer val for encoder
	__HAL_TIM_SetAutoreload(&htim8,OPTS_MAIN_MENU-1);
	print_menu(main_menu_opts, OPTS_MAIN_MENU);
	setCursor(cursorLine);
	while (1)
	{
		if(cursorLine != last_cursorline){
			delCursor(last_cursorline);
			setCursor(cursorLine);
			last_cursorline = cursorLine;
		}
		if(btn_enc){
			HAL_Delay(300);
			//HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
			btn_enc = 0;
			switch (cursorLine)
			{
			case 0:
				menu_image_processing();
				break;
			case 1:
				// TODO: call function
				break;
			case 2:
				// TODO: call function
				break;
			default:break;
			}
			// when back in this menu -> call print menu function

			htim8.Instance->CNT = cursorLine = last_cursorline = 0;
			__HAL_TIM_SetAutoreload(&htim8,OPTS_MAIN_MENU-1);
			print_menu(main_menu_opts, OPTS_MAIN_MENU);
			setCursor(cursorLine);
		}
	}
}
