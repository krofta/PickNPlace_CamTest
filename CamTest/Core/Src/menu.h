/*
 * menu.h
 *
 *  Created on: Dec 30, 2020
 *      Author: niko
 */


#ifndef SRC_MENU_H_
#define SRC_MENU_H_

#include <inttypes.h>

#define MENU_LINE_HEIGHT 20
#define MENU_LINE_WIDTH 320

#define KEY_ARROW_UP 0
#define KEY_ARROW_DOWN 2
#define KEY_ENTER 3


#define ACT_MENU_MAIN 0
#define ACT_MENU_IMAGE_PROCESSING 1
#define ACT_MENU_PREPROCESSING 401
#define ACT_MENU_EDGE_DETECTION 501
#define ACT_MENU_LIGHT 10

#define OPTS_MAIN_MENU 	3
#define OPTS_IMAGE_PROCESSING 8
#define OPTS_PREPROCESSING 8
#define OPTS_EDGE_DETECTION 6

void setCursor(int pos);
void delCursor(int pos);
void print_menu(char *options[], int opt_count);
void menu();
void image_processing_menu(int opts);





#endif /* SRC_MENU_H_ */
