/*
 * sensor.c
 *
 *  Created on: Jul 28, 2023
 *      Author: Zain Irsyad
 */

#include "sensor.h"
#include "stdlib.h"
#include "memory.h"


ADC_HandleTypeDef* sen_hadc;

uint16_t adc_sensor[SENSOR_NUM];
//uint16_t abnorm_sensor[SENSOR_NUM];
uint16_t adc_sensor_arranged[SENSOR_NUM];
uint16_t threshold_sensor[SENSOR_NUM];

uint8_t sensor_disp_status[SENSOR_NUM];
uint16_t sensor_binary;

uint8_t sensor_invert = 0;

#define ADC_FILTER_LEN 3
uint16_t adc_filter_mem[SENSOR_NUM][ADC_FILTER_LEN];;
uint8_t adc_filter_pt = 0;

uint8_t sensor_flag = 0;

void SEN_Init(ADC_HandleTypeDef* hadc) {
	sen_hadc = hadc;
	for(int i=0; i<SENSOR_NUM; i++)  threshold_sensor[i] = ADC_MAX/2;
}

void SEN_Filter(uint16_t* data) {
	uint16_t temp[SENSOR_NUM];
	for(int i=0; i<SENSOR_NUM; i++) {
		adc_filter_mem[i][adc_filter_pt] = data[i];
		if(adc_filter_pt<ADC_FILTER_LEN-1) adc_filter_pt++; else adc_filter_pt=0;
		memset((uint8_t*)temp, 0, SENSOR_NUM*2);
		for(int j=0; j<ADC_FILTER_LEN; j++) {
			temp[i] += adc_filter_mem[i][j]/ADC_FILTER_LEN;
		}
		data[i] = temp[i];
	}
}

void SEN_StartReadSensor() {
	HAL_ADC_Start_DMA(sen_hadc, adc_sensor, SENSOR_NUM);
}

void SEN_CallbackRoutine(ADC_HandleTypeDef* hadc) {
	if(hadc->Instance != sen_hadc->Instance) return;
	SEN_Filter(adc_sensor);
	for(int i=0; i<SENSOR_NUM/2; i++) {
		adc_sensor_arranged[SENSOR_NUM-i-1] = adc_sensor[i*2];
		adc_sensor_arranged[i] = adc_sensor[i*2+1];
	}
	sensor_binary = 0;
	for(int i=0; i<SENSOR_NUM; i++) {
		uint8_t bit = 0;
		if(sensor_invert == 0) {
			if(adc_sensor_arranged[i] > threshold_sensor[i]) {
				bit = 1;
				sensor_disp_status[i] = (adc_sensor_arranged[i]-threshold_sensor[i])/SEN_BAR_INC + 1;
			}
			else sensor_disp_status[i] = 0;
		}
		else {
			if(adc_sensor_arranged[i] < threshold_sensor[i]) {
				bit = 1;
				sensor_disp_status[i] = (threshold_sensor[i]-adc_sensor_arranged[i])/SEN_BAR_INC + 1;
			}
			else sensor_disp_status[i] = 0;
		}

		sensor_binary |= bit << (SENSOR_NUM-i-1);
	}

	if(autocalib_status == 1) {
		SEN_AutoCalibrationRoutine(0);
	}

	sensor_flag |= 1;
	sensor_flag &=~2;
}

uint16_t* calib_upper_adc;
uint16_t* calib_lower_adc;
uint8_t autocalib_status = 0;

void SEN_StartAutoCalibration() {
	if(autocalib_status == 1) return;
	calib_upper_adc = (uint16_t*)malloc(sizeof(uint16_t)*SENSOR_NUM);
	calib_lower_adc = (uint16_t*)malloc(sizeof(uint16_t)*SENSOR_NUM);
	if(calib_upper_adc || calib_lower_adc) {
		for(int i=0; i<SENSOR_NUM; i++) {
			calib_upper_adc[i] = 0x000;
			calib_lower_adc[i] = ADC_MAX;
		}
		autocalib_status = 1;
	}
	else autocalib_status = 0;
}

void SEN_AutoCalibrationRoutine(uint8_t flag) {
	for(int i=0; i<SENSOR_NUM; i++) {
		if(calib_upper_adc[i] < adc_sensor_arranged[i])
			calib_upper_adc[i] = adc_sensor_arranged[i];
		else if(calib_lower_adc[i] > adc_sensor_arranged[i])
			calib_lower_adc[i] = adc_sensor_arranged[i];
	}
}

void SEN_EndAutoCalibration(uint8_t flag) {
	if(autocalib_status == 0) return;
	if(flag & 1) {
		for(int i=0; i<SENSOR_NUM; i++) {
			if(calib_upper_adc[i] < calib_lower_adc[i]) continue;
			threshold_sensor[i] = TH_CALIB_FACTOR * (calib_upper_adc[i]-calib_lower_adc[i]) + calib_lower_adc[i];
		}
	}
	free(calib_upper_adc);
	free(calib_lower_adc);
	autocalib_status = 0;
}

void SEN_BlackLine() {
	sensor_invert = 0;
}

void SEN_WhiteLine() {
	sensor_invert = 1;
}

void SEN_Invert() {
	sensor_invert = !sensor_invert;
}
