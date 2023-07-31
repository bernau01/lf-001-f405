/*
 * oled.c
 *
 *  Created on: Jul 21, 2023
 *      Author: Zain Irsyad
 */

#include "oled.h"

I2C_HandleTypeDef* oled_hi2c;

void OLED_Command(uint8_t byte)
{
	HAL_I2C_Mem_Write(oled_hi2c, OLED_ADDR, 0x00, 1, &byte, 1, 10);
}

void OLED_DoubleCommand(uint8_t byte1, uint8_t byte2)
{
	OLED_Command(byte1);
	OLED_Command(byte2);
}

void OLED_Data(uint8_t byte)
{
	HAL_I2C_Mem_Write(oled_hi2c, OLED_ADDR, 0x40, 1, &byte, 1, 10);
}

void OLED_CmdData(uint8_t cmdByte, uint8_t dataByte)
{
	uint8_t cmdBuff[] = {0x80, cmdByte, 0x40, dataByte};
	HAL_I2C_Master_Transmit(oled_hi2c, OLED_ADDR, cmdBuff, 4, 10);
}

void OLED_MultipleData(uint8_t *arrData, uint8_t arrSize)
{
	HAL_I2C_Mem_Write(oled_hi2c, OLED_ADDR, 0x40, 1, arrData, arrSize, 10);
}

void OLED_Inverse(uint8_t mode) {
	OLED_Command(0xA6 + (mode>=1));
}

void OLED_SetPageAddr(uint8_t addr) {
	OLED_Command(OLED_PAGE_ADDR | (addr & 0x07));
}

void OLED_SetColumn(uint8_t col) {
	col += OLED_X_OFFSET;
	OLED_DoubleCommand(0x00 | (col & 0x0F), 0x10 | (col >> 4));
}

void OLED_Test() {
	for(int j=0; j<OLED_HEIGHT/8; j++) {
		OLED_SetPageAddr(j);
		OLED_SetColumn(0);
		for(int i=0; i<OLED_WIDTH; i++) {
			OLED_Data(0xFF);
			HAL_Delay(10);
		}
		OLED_SetColumn(0);
		for(int i=0; i<OLED_WIDTH; i++) {
			OLED_Data(0x00);
			HAL_Delay(10);
		}
	}
}

void OLED_ClearRAM() {
	for(int j=0; j<8; j++) {
		OLED_SetPageAddr(j);
		OLED_SetColumn(0);
			for(int i=0; i<OLED_WIDTH; i++) {
				OLED_Data(0x00);
			}
		}
}

//void OLED_Load()
//{
//	uint8_t i, ed = 0xA4;
//	for (i=0; i<8; i++)
//	{
//		OLED_Command(0xB0 + i);
//		OLED_Command(0x00);
//		OLED_Command(0x10);
//		OLED_MultipleData(&ed, 130);
//	}
//}

void OLED_Init(I2C_HandleTypeDef* hi2c)
{
	oled_hi2c = hi2c;

	OLED_Command(OLED_DISP_OFF);
	OLED_DoubleCommand(OLED_MUX, 0x3F);
	OLED_DoubleCommand(OLED_OFFSET, 0x00);
	OLED_Command(OLED_START_LINE);
	OLED_Command(OLED_SEG_RMAP | 0x01);
	OLED_Command(OLED_SCAN_DIR | 0x08);
	OLED_DoubleCommand(OLED_COM_PADS,OLED_COM_PADS_MODE|0x10);
	OLED_DoubleCommand(OLED_CONTRAST, 0x60);
	OLED_Command(OLED_FORCE_ON);
	OLED_Command(OLED_DISP_RV | 0x00);
	OLED_DoubleCommand(OLED_CLK_OSC, 0xF0);
//	OLED_Command(OLED_VPP_8V0);
	OLED_ClearRAM();
	OLED_Command(OLED_DISP_ON);
//	OLED_Load();
}
