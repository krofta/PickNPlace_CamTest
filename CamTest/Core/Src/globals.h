/*
 * globals.h
 *
 *  Created on: Dec 30, 2020
 *      Author: niko
 */

#ifndef SRC_GLOBALS_H_
#define SRC_GLOBALS_H_
//#include "bv.h"

#define OV7670_QVGA_WIDTH  320
#define OV7670_QVGA_HEIGHT 240

#define MAXXDIM 320
#define MAXYDIM 240
#define MAXBLOBS 10000
#define PIXEL_DEPTH 256
#define HISTO_KUMULATIV 1
#define HISTO_NORMAL 0

#define RAM 320000
// 153600 bytes
extern uint16_t framebuffer[OV7670_QVGA_HEIGHT ][OV7670_QVGA_WIDTH];
// 76800 bytes
extern unsigned char img[MAXYDIM ][MAXXDIM];


// 9 bytes
extern uint8_t btn_enc;
extern uint8_t pic_captured;
extern uint8_t pic_written;
extern uint8_t led_val;
extern uint8_t led_val_changed;
extern uint16_t cursorLine;
extern uint16_t actual_menu;
// Framebuffer for capuring images and writing the display




#endif /* SRC_GLOBALS_H_ */
