/*
 * menu.c
 *
 *  Created on: Dec 30, 2020
 *      Author: niko
 */

#include "menu.h"
#include "st7789.h"

uint8_t actual_menu = ACT_MENU_MAIN;
char *main_menu_opts[MAIN_MENU_OPTS] = {
		"Image processing",
		"Light settings",
		"Camera settings"
};
uint16_t cursorLine = 0;

// extern variables
extern TIM_HandleTypeDef htim8;
extern uint8_t btn_enc;


void setCursor(int pos){
	ST7789_DrawRectangle(0, pos * MENU_LINE_HEIGHT, MENU_LINE_WIDTH - 1,
			pos * MENU_LINE_HEIGHT + MENU_LINE_HEIGHT, WHITE);
}
void delCursor(int pos){
	ST7789_DrawRectangle(0, pos * MENU_LINE_HEIGHT, MENU_LINE_WIDTH - 1,
			pos * MENU_LINE_HEIGHT + MENU_LINE_HEIGHT, BLACK);
}


void print_main_menu(int cursorline, int opts)
{
	ST7789_Fill_Color(BLACK);
	for(int i = 0; i < opts; i++){
		ST7789_WriteString(10, i*MENU_LINE_HEIGHT, main_menu_opts[i], Font_11x18, WHITE, BLACK);
	}
}


void menu()
{
	uint16_t last_cursorline;

	// set global variable, that actual manu is main menu
	actual_menu = ACT_MENU_MAIN;
	// set encoder count to zero
	htim8.Instance->CNT = 0;
	last_cursorline = htim8.Instance->CNT;
	// set max timer val for encoder
	__HAL_TIM_SetAutoreload(&htim8,MAIN_MENU_OPTS-1);

	print_main_menu(cursorLine, MAIN_MENU_OPTS);
	setCursor(cursorLine);
	while (1)
	{

		if(cursorLine != last_cursorline){
			delCursor(last_cursorline);
			setCursor(cursorLine);
			last_cursorline = cursorLine;
			if(btn_enc){
				HAL_Delay(150);
				HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
				btn_enc = 0;
				switch (cursorLine)
				{
				case 0:
					// TODO: call function
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
				print_main_menu(cursorLine, MAIN_MENU_OPTS);

			}

		}
	}
}
