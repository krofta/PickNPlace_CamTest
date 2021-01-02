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
#include "bv.h"
uint16_t actual_menu = ACT_MENU_MAIN;
uint16_t cursorLine = 0;

// extern variables
extern TIM_HandleTypeDef htim8;
extern uint8_t btn_enc;

uint16_t segmentierung_von_otsu(unsigned char img[MAXYDIM][MAXXDIM]);
void segmentierung_binaer(unsigned char img[MAXYDIM][MAXXDIM], uint16_t threshold);
void invert(unsigned char img[MAXYDIM][MAXXDIM]);
//void blob_coloring_imagesensitiv(unsigned char img[MAXYDIM][MAXXDIM], unsigned char img2[MAXYDIM][MAXXDIM], int iIMG[MAXYDIM][MAXXDIM],
//		int iteration, int keine_fransen, int writeImage, int iterationen);
uint16_t find_blobs(unsigned char img[MAXYDIM][MAXXDIM], uint16_t iIMG[MAXYDIM][MAXXDIM], uint16_t bereich);
uint16_t blob_coloring_markersensitiv(unsigned char img[MAXYDIM][MAXXDIM], uint16_t iIMG[MAXYDIM][MAXXDIM], int bereich, int writeImage);
void biggestBlob(unsigned char img[MAXYDIM][MAXXDIM],uint16_t iIMG[MAXYDIM][MAXXDIM], uint16_t background_threshold, uint16_t min_blobsize);


char *segmentation_menu_opts[OPTS_SEGMENTATION] = {
		"Seg. von Otsu",
		"Seg.binaer",
		"Invert",
		"Blob segmentation",
		"Find biggest blob",
		"Back"
};

char *edge_detection_menu_opts[OPTS_EDGE_DETECTION] = {
		"Sobel operator x",
		"Sobel operator y",
		"Sobel operator xy",
		"Laplace operator",
		"Difference of gaussian",
		"Back"
};


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
	rgb_to_greyscale(framebuffer, img);
	if(manu){
		ST7789_WriteString(10, 0, "Press to continue", Font_11x18, WHITE, BLACK);
		while(!btn_enc);
		HAL_Delay(400);
		btn_enc = 0;
	}
	return;
}

void show_image(int manu){

	// GREEN 0x003F
	// RED   0x07C0
	// BLUE

	greyscale_to_greyrgb(framebuffer, img);
	ST7789_DrawImage(0,0,320,240,framebuffer);
	if(manu){
		ST7789_WriteString(10, 0, "Press to continue", Font_11x18, WHITE, BLACK);
		while(!btn_enc);
		HAL_Delay(400);
		btn_enc = 0;
	}
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
	init_iMatrix(framebuffer, BLACK);
	ST7789_DrawImage(0,0,320,240,framebuffer);
	for(int i = 0; i < opt_count; i++){
		ST7789_WriteString(10, i*MENU_LINE_HEIGHT, options[i], Font_11x18, WHITE, BLACK);
	}
}
void menu_segmentation(){
	uint16_t last_cursorline;
	//HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	// set global variable, that actual manu is main menu
	actual_menu = ACT_MENU_SEGMENTATION;
	// set encoder count to zero
	htim8.Instance->CNT = 0;
	last_cursorline = htim8.Instance->CNT;
	// set max timer val for encoder
	__HAL_TIM_SetAutoreload(&htim8,OPTS_SEGMENTATION-1);
	print_menu(segmentation_menu_opts, OPTS_SEGMENTATION);
	setCursor(cursorLine);
	char buf[10];
	uint16_t blobs = 0;
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
				segmentierung_von_otsu(img);
				show_image(1);
				break;
			case 1:
				segmentierung_binaer(img, 75);
				show_image(1);
				break;
			case 2:
				invert(img);
				show_image(1);
				break;
			case 3:
				blobs = blob_coloring_markersensitiv(img, framebuffer, 10, 1);
				sprintf(buf,"%hu",blobs);
				show_image(1);
				ST7789_WriteString(10, 20, buf, Font_11x18, WHITE, BLACK);
				break;
			case 4:
				biggestBlob(img, framebuffer, 200, 150);
				show_image(1);
				break;
			case 5:
				return;

			default:break;
			}
			// when back in this menu -> call print menu function
			htim8.Instance->CNT = cursorLine = last_cursorline = 0;
			__HAL_TIM_SetAutoreload(&htim8,OPTS_SEGMENTATION-1);
			print_menu(segmentation_menu_opts, OPTS_SEGMENTATION);
			setCursor(cursorLine);
		}
	}
}

void menu_edge_detection(){
	uint16_t last_cursorline;
	//HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	// set global variable, that actual manu is main menu
	actual_menu = ACT_MENU_EDGE_DETECTION;
	// set encoder count to zero
	htim8.Instance->CNT = 0;
	last_cursorline = htim8.Instance->CNT;
	// set max timer val for encoder
	__HAL_TIM_SetAutoreload(&htim8,OPTS_EDGE_DETECTION-1);
	print_menu(edge_detection_menu_opts, OPTS_EDGE_DETECTION);
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
				sobelx(img, framebuffer);
				show_image(1);
				break;
			case 1:
				sobely(img, framebuffer);
				show_image(1);
				break;
			case 2:
				break;
			case 3:
				laplace(img, framebuffer, LAPLACE_8);
				show_image(1);
				break;
			case 4:
				difference_of_gaussian(img, framebuffer, 3, 0);
				show_image(1);
				break;
			case 5:
				return;

			default:break;
			}
			// when back in this menu -> call print menu function
			htim8.Instance->CNT = cursorLine = last_cursorline = 0;
			__HAL_TIM_SetAutoreload(&htim8,OPTS_EDGE_DETECTION-1);
			print_menu(edge_detection_menu_opts, OPTS_EDGE_DETECTION);
			setCursor(cursorLine);
		}
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
				histogramm(img,HISTO_NORMAL);
				show_image(1);
				break;
			case 1:
				histogramm(img,HISTO_KUMULATIV);
				show_image(1);
				break;
			case 2:
				grauwert_dehnung(img);
				show_image(1);
				break;
			case 3:
				linearer_histo_ausgleich(img, 32);
				show_image(1);
				break;
			case 4:
				mittelwert_filter(img, img2, 3, 1);
				show_image(1);
				break;
			case 5:
				median_filter3x3(img, img2);
				show_image(1);
				break;
			case 6:
				//gauss_filter(img, img2, 3);
				// TODO: call function
				break;
			case 7:
				return;
			default:break;
			}
			// when back in this menu -> call print menu function
			htim8.Instance->CNT = cursorLine = last_cursorline = 0;
			__HAL_TIM_SetAutoreload(&htim8,OPTS_PREPROCESSING-1);
			print_menu(preprocessing_menu_opts, OPTS_PREPROCESSING);
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
				break;
			case 1:
				show_image(1);
				break;
			case 2:
				// TODO: call function
				break;
			case 3:
				menu_pre_processing();
				break;
			case 4:
				menu_edge_detection();
				// TODO: call function
				break;
			case 5:
				// TODO: call function
				break;
			case 6:
				menu_segmentation();
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
