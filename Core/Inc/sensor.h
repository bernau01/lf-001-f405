/*
 * sensor.h
 *
 *  Created on: Jul 28, 2023
 *      Author: Zain Irsyad
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include "main.h"

#define SEN_BAR_INC 10
#define SENSOR_NUM 14
#define ADC_MAX 4095
#define TH_CALIB_FACTOR 0.4

#define SENSOR_FLAG_DONE		0x01
#define SENSOR_FLAG_POLLING		0x02

extern uint16_t adc_sensor[SENSOR_NUM];

extern uint16_t adc_sensor_arranged[SENSOR_NUM];
extern uint16_t threshold_sensor[SENSOR_NUM];

extern uint8_t sensor_disp_status[SENSOR_NUM];
extern uint16_t sensor_binary;

extern uint8_t sensor_flag;
extern uint8_t autocalib_status;

void SEN_Init(ADC_HandleTypeDef* hadc);
void SEN_StartReadSensor();
void SEN_CallbackRoutine(ADC_HandleTypeDef* hadc);
void SEN_AutoCalibrationRoutine(uint8_t flag);
void SEN_StartAutoCalibration();
void SEN_EndAutoCalibration(uint8_t flag);
void SEN_Invert();
void SEN_BlackLine();
void SEN_WhiteLine();

#endif /* INC_SENSOR_H_ */
