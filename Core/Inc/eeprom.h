/*
 * eeprom.h
 *
 *  Created on: Jul 30, 2023
 *      Author: Zain Irsyad
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "main.h"

#define DEV_ADDR		0xA0

#define PAGE_SIZE		64
#define PAGE_NUM		512
#define PAGE_BIT_POS	6

#define WRITE_WAIT_TIME 8//ms

void EEP_Init(I2C_HandleTypeDef* hi2c);
void EEP_WritePage(uint16_t page_addr, uint8_t* data);
void EEP_WriteMem(uint16_t mem_addr, uint8_t* data, uint8_t size);
void EEP_ReadPage(uint16_t page_addr, uint8_t* data);
void EEP_ReadMem(uint16_t mem_addr, uint8_t* data, uint8_t size);
uint8_t EEP_ReadyToWrite();

#endif /* INC_EEPROM_H_ */
