/*
 * oled.h
 *
 *  Created on: Jul 21, 2023
 *      Author: Zain Irsyad
 */

#ifndef INC_OLED_H_
#define INC_OLED_H_

#include "main.h"

#define OLED_WIDTH 128
#define OLED_HEIGHT	64

#define OLED_X_OFFSET 2

#define OLED_ADDR   			(0x3C << 1)

#define OLED_LOW_COLUMN			0x00
#define OLED_HIGH_COLUMN		0x10

#define OLED_VPP_6V4			0x30
#define OLED_VPP_7V4			0x31
#define OLED_VPP_8V0			0x32
#define OLED_VPP_9V0			0x33

#define OLED_START_LINE			0x40

#define OLED_CONTRAST			0x81

#define OLED_SEG_RMAP			0xA0

#define OLED_FORCE_ON			0xA4

#define OLED_DISP_RV			0xA6

#define OLED_MUX				0xA8

#define OLED_DC_MODE			0xAD

#define OLED_DISP_ON			0xAF
#define OLED_DISP_OFF			0XAE
#define OLED_DC_STAT			0x8A

#define OLED_PAGE_ADDR			0xB0

#define OLED_SCAN_DIR			0xC0

#define OLED_OFFSET				0xD3

#define OLED_CLK_OSC			0xD5

#define OLED_PRECHRG			0xD9

#define OLED_COM_PADS			0xDA
#define OLED_COM_PADS_MODE 		0x02

#define OLED_VCOM_DS			0xDB

#define OLED_READ_MODIFY_WRITE	0xE0

#define OLED_END				0xEE

#define OLED_NOP				0xE3

void OLED_Command(uint8_t byte);
void OLED_DoubleCommand(uint8_t byte1, uint8_t byte2);
void OLED_Data(uint8_t byte);
void OLED_CmdData(uint8_t cmdByte, uint8_t dataByte);
void OLED_MultipleData(uint8_t *arrData, uint8_t arrSize);
void OLED_Init(I2C_HandleTypeDef* hi2c);
void OLED_Inverse(uint8_t mode);
void OLED_SetPageAddr(uint8_t addr);
void OLED_SetColumn(uint8_t col);
void OLED_ClearRAM();
void OLED_Test();

#endif /* INC_OLED_H_ */
