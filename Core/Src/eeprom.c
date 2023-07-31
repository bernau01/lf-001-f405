/*
 * eeprom.c
 *
 *  Created on: Jul 30, 2023
 *      Author: Zain Irsyad
 */

#include "eeprom.h"

I2C_HandleTypeDef* eep_hi2c;

uint32_t last_tick = 0;

uint16_t last_mem;

void EEP_Init(I2C_HandleTypeDef* hi2c) {
	eep_hi2c = hi2c;
}

void EEP_WritePage(uint16_t page_addr, uint8_t* data) {
	uint16_t mem_addr = page_addr << PAGE_BIT_POS;
	HAL_I2C_Mem_Write(eep_hi2c, DEV_ADDR, mem_addr, 2, data, PAGE_SIZE, 100);
	last_tick = HAL_GetTick();
}

void EEP_WriteMem(uint16_t mem_addr, uint8_t* data, uint8_t size) {
	HAL_I2C_Mem_Write(eep_hi2c, DEV_ADDR, mem_addr, 2, data, size, 100);
	last_tick = HAL_GetTick();
}

void EEP_ReadPage(uint16_t page_addr, uint8_t* data) {
	uint16_t mem_addr = page_addr << PAGE_BIT_POS;
	HAL_I2C_Mem_Read(eep_hi2c, DEV_ADDR, mem_addr, 2, data, PAGE_SIZE, 100);
	last_mem = mem_addr+PAGE_SIZE;
}

void EEP_ReadMem(uint16_t mem_addr, uint8_t* data, uint8_t size) {
	HAL_I2C_Mem_Read(eep_hi2c, DEV_ADDR, mem_addr, 2, data, size, 100);
	last_mem = mem_addr+size;
}

uint8_t EEP_ReadyToWrite() {
	if(HAL_GetTick() - last_tick > 5) return 1; else return 0;
}
