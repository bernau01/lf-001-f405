/*
 * motor.c
 *
 *  Created on: Jul 28, 2023
 *      Author: Zain Irsyad
 */

#include "motor.h"

void Motor_Init(Motor_typedef* hmot) {

}

void Motor_SetPWM(Motor_typedef* hmot, float value) {
	if(value > 99) value = 99;
	else if(value < -99) value = -99;
	hmot->pwm = value;
}

void Motor_SetVel(Motor_typedef* hmot, float value) {
	value *= hmot->vel_factor;
	hmot->vel_sp = value;
}

void Motor_SetPointF(Motor_typedef* hmot, float value) {
	switch(hmot->mode) {
	case MOTOR_MODE_OPEN:
		Motor_SetPWM(hmot, value);
		break;
	case MOTOR_MODE_CLOSE:
		Motor_SetVel(hmot, value);
		break;
	}
}

void Motor_SetPoint(Motor_typedef* hmot, float value) {
	switch(hmot->mode) {
	case MOTOR_MODE_OPEN:
		Motor_SetPWM(hmot, value);
		break;
	case MOTOR_MODE_CLOSE:
		Motor_SetVel(hmot, value);
		break;
	}
}

void Motor_ApplyPWM(Motor_typedef* hmot) {
	int16_t tim_period = (hmot->pwm*hmot->pwm_factor)*hmot->pwm_htim->Init.Period;
	uint8_t pwm_sign = tim_period < 0;
//	uint8_t dir_sign = hmot->dir < 0;

	if(pwm_sign) tim_period*=-1;
	if(tim_period > hmot->pwm_htim->Init.Period) {
		tim_period = (hmot->pwm_htim->Init.Period);
	}

	if(pwm_sign) {
		__HAL_TIM_SET_COMPARE(hmot->pwm_htim, hmot->ch1, 0);
		__HAL_TIM_SET_COMPARE(hmot->pwm_htim, hmot->ch2, tim_period);
	}
	else {
		__HAL_TIM_SET_COMPARE(hmot->pwm_htim, hmot->ch1, tim_period);
		__HAL_TIM_SET_COMPARE(hmot->pwm_htim, hmot->ch2, 0);
	}
}

void Motor_EnocderRoutine(Motor_typedef* hmot, float period) {
	int16_t tim_cnt_now = __HAL_TIM_GET_COUNTER(hmot->enc_htim) * hmot->dir;
	hmot->enc_vel =  tim_cnt_now - hmot->enc_cnt;
	hmot->enc_cnt = tim_cnt_now;
}

void Motor_ControlRoutine(Motor_typedef* hmot, float __period) {
	float error, mv;
	switch(hmot->mode) {
	case MOTOR_MODE_CLOSE:
		error = hmot->vel_sp - hmot->enc_vel;
		if(hmot->vel_sp == 0) hmot->sum_error=0;
		if((hmot->vel_sp > 0 && hmot->pwm < 0)||(hmot->vel_sp < 0 && hmot->pwm > 0)) hmot->sum_error=0;
		mv = (hmot->kp * error) + (hmot->ki*hmot->sum_error*__period);
		if(mv>90) {
			hmot->sum_error*=0.9;
		}
		else {
			hmot->sum_error += error;
		}
		hmot->pwm = mv;
		hmot->last_error = error;

		hmot->filter = hmot->filter*(1-hmot->filter_alpha) + hmot->pwm*hmot->filter_alpha;
		hmot->pwm = hmot->filter;

	case MOTOR_MODE_OPEN:
		Motor_ApplyPWM(hmot);
	}
}
