/*
 * motor.h
 *
 *  Created on: Jul 28, 2023
 *      Author: Zain Irsyad
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

typedef enum {
	MOTOR_MODE_OPEN,
	MOTOR_MODE_CLOSE
} Motor_Mode_typedef;

typedef struct {
	Motor_Mode_typedef mode;
	TIM_HandleTypeDef* pwm_htim;
	uint16_t ch1;
	uint16_t ch2;
	int8_t dir;
	TIM_HandleTypeDef* enc_htim;
	int16_t enc_last_cnt;
	int32_t enc_cnt;
	int32_t enc_vel;
	float pwm;
	float pwm_factor;
	int16_t vel_sp;
	float vel_factor;
	float kp;
	float ki;
	float last_error;
	float sum_error;
	float filter;
	float filter_alpha;
} Motor_typedef;

void Motor_SetPoint(Motor_typedef* hmot, float value);
void Motor_ApplyPWM(Motor_typedef* hmot);
void Motor_EnocderRoutine(Motor_typedef* hmot, float period);
void Motor_ControlRoutine(Motor_typedef* hmot, float period);
void Run_MotorRoutine(float period);
void Run_MotorRoutine2(float _period);
#endif /* INC_MOTOR_H_ */
