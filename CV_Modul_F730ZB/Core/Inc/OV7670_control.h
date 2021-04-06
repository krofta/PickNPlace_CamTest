/*
*	==========================================================================
*   OV7670_control.h	
*   (c) 2014, Petr Machala
*
*   Description:
*   OV7670 camera configuration and control library.
*   Optimized for 32F429IDISCOVERY board.
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   any later version.
*   
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*  
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*   Camera wiring:
*   3V3		-	3V		;		GND		-	GND
*   SIOC	-	PB8		;		SIOD	-	PB9
*   VSYNC -	PB7			;		HREF	-	PA4
*   PCLK	-	PA6		;		XCLK	-	PA8
*   D7		-	PE6		;		D6		-	PE5
*   D5		-	PB6		;		D4		-	PE4
*   D3		-	PC9		;		D2		-	PC8
*   D1		-	PC7		;		D0		-	PC6
*   RESET	-	/			;		PWDN	-	/
*		
*	==========================================================================
*/
#include "main.h"

/*
* Initialize SCCB
*/
uint16_t CAMERA_IO_Read(I2C_HandleTypeDef *hi2c, uint8_t Addr, uint16_t Reg);
void CAMERA_IO_Write(I2C_HandleTypeDef *hi2c, uint8_t Addr, uint16_t Reg, uint16_t Value);

static HAL_StatusTypeDef I2Cx_ReadMultiple(I2C_HandleTypeDef *hi2c, uint8_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length);
static HAL_StatusTypeDef I2Cx_WriteMultiple(I2C_HandleTypeDef *hi2c, uint8_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length);
extern void SCCB_init(void);
extern int OV7670_init(I2C_HandleTypeDef *hi2c);
//int SCCB_write_reg(uint8_t reg_addr, uint8_t* data);



