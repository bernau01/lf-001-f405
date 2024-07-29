/*
 * run.c
 *
 *  Created on: Jul 29, 2023
 *      Author: Zain Irsyad
 */

#include "run.h"
#include "memory.h"
#include "bindef.h"
#include "math.h"


Motor_typedef motor[2];

int32_t robot_enc_pos = 0;
float robot_enc_yawpos = 0;

float temp_yaw_speed;
float this_accl;

void Run_Init() {
	memset(motor, 0, sizeof(motor[0])*2);

	motor[0].mode = MOTOR_MODE_CLOSE;
	motor[0].pwm_htim = &htim3;
	motor[0].ch1 = TIM_CHANNEL_1;
	motor[0].ch2 = TIM_CHANNEL_2;
	motor[0].enc_htim = &htim2;
	motor[0].dir = 1;
	motor[0].pwm_factor = 0.01;
	motor[0].vel_factor = 20;
	motor[0].kp = 0.2;
	motor[0].ki = 5;
	motor[0].filter_alpha = 0.5;

	motor[1].mode = motor[0].mode;
	motor[1].pwm_htim = &htim3;
	motor[1].ch1 = TIM_CHANNEL_3;
	motor[1].ch2 = TIM_CHANNEL_4;
	motor[1].enc_htim = &htim4;
	motor[1].dir = -1;
	motor[1].pwm_factor = 0.0097;
	motor[1].vel_factor = 20;
	motor[1].kp = 0.2;
	motor[1].ki = motor[0].ki;
	motor[1].filter_alpha = motor[0].filter_alpha;

	robot_enc_pos = 0;
}

void Run_MotorRoutine(float period) {
	Motor_EnocderRoutine(&motor[0], period);
	Motor_EnocderRoutine(&motor[1], period);
	robot_enc_pos += (motor[0].enc_vel*0.5) + (motor[1].enc_vel*0.5);
	robot_enc_yawpos += (motor[1].enc_vel - motor[0].enc_vel);
//	Motor_ControlRoutine(&motor[0], period);
//	Motor_ControlRoutine(&motor[1], period);
}

void Run_MotorRoutine2(float _period) {
//	Motor_EnocderRoutine(&motor[0], period);
//	Motor_EnocderRoutine(&motor[1], period);
	Motor_ControlRoutine(&motor[0], _period);
	Motor_ControlRoutine(&motor[1], _period);
}


float last_d;
float kp;
float ki;
float kd;
float alpha;
int16_t sum_error;

float Run_YawSpeed(float period, float _speed, uint8_t flag) {
	static int16_t last_error = 0;
	static int16_t last_div_error = 0;
	static float last_mv = 0;
	int16_t error = 0;

	uint16_t pr_sensor_bin = 0;
	uint16_t mask = (1<<SENSOR_NUM)-1;
	if((flag&0x01)==0x01) {
		pr_sensor_bin = (~sensor_binary) & mask;
	}
	else {
		pr_sensor_bin = sensor_binary & mask;
	}

	switch(sensor_binary) {
	case B00000000000001: error = -12; break;
	case B00000000000011: error = -11; break;
//	case B00000000000011: error = -11; break;
	case B00000000000111: error = -10; break;
	case B00000000001110: error = -9; break;
	case B00000000001100: error = -8; break;
	case B00000000011100: error = -7; break;
	case B00000000011000: error = -6; break;
	case B00000000111000: error = -5; break;
	case B00000000110000: error = -4; break;
	case B00000001110000: error = -3; break;
	case B00000001100000: error = -2; break;
	case B00000011100000: error = -1; break;

	case B00000011000000: error = 0; break;

	case B00000111000000: error = 1; break;
	case B00000110000000: error = 2; break;
	case B00001110000000: error = 3; break;
	case B00001100000000: error = 4; break;
	case B00011100000000: error = 5; break;
	case B00011000000000: error = 6; break;
	case B00111000000000: error = 7; break;
	case B00110000000000: error = 8; break;
	case B01110000000000: error = 9; break;
	case B11100000000000: error = 10; break;
//	case B11000000000000: error = 11; break;
	case B11000000000000: error = 11; break;
	case B10000000000000: error = 12; break;

	case B00000011110000: error = -1; break;
	case B00000111111000: error = -1; break;
	case B00001111111100: error = -1; break;
	case B00011111111110: error = -1; break;

	case B00000111100000: error = 0; break;
	case B00001111110000: error = 0; break;
	case B00011111111000: error = 0; break;
	case B00111111111100: error = 0; break;

	case B00001111000000: error = 1; break;
	case B00011111100000: error = 1; break;
	case B00111111110000: error = 1; break;
	case B01111111111000: error = 1; break;

	case B11111000000001: error = -2; break;

	case B11100000000001: error = -1; break;
	case B11110000000011: error = -1; break;

	case B10000000000001: error = 0; break;
	case B11000000000011: error = 0; break;
	case B11100000000111: error = 0; break;

	case B10000000000111: error = -1; break;
	case B11000000001111: error = -1; break;

	case B10000000011111: error = -2; break;

	case B11000001100000: error = -2; break;
	case B10000011100000: error = -1; break;
	case B10000011000001: error = 0; break;
	case B00000111000001: error = 1; break;
	case B10000110000011: error = 2; break;

	case B11100001100000: error = -2; break;
	case B01100001100000: error = -2; break;
//	case B11000011110000: error = -2; break;
	case B10000001100000: error = -2; break;

	case B11100011100000: error = -1; break;
	case B11000011100000: error = -1; break;
//	case B10000111110000: error = -1; break;

	case B01100011000000: error = 0; break;
	case B11100011000000: error = 0; break;
	case B11000011000000: error = 0; break;
	case B10000011000000: error = 0; break;

	case B01100111000000: error = 1; break;
	case B11100111000000: error = 1; break;
	case B11000111000000: error = 1; break;
	case B10000111000000: error = 1; break;

	case B01100110000000: error = 2; break;
	case B11100110000000: error = 2; break;
	case B11000110000000: error = 2; break;
	case B10000110000000: error = 2; break;

	case B10001110000000: error = 3; break;

	case B10001100000000: error = 4; break;



	case B00000110000110: error = 2; break;
	case B00000110000111: error = 2; break;
	case B00000110000011: error = 2; break;
	case B00000110000001: error = 2; break;

	case B00000111000111: error = 1; break;
	case B00000111000011: error = 1; break;
//	case B00001111100001: error = 1; break;

	case B00000011000110: error = 0; break;
	case B00000011000111: error = 0; break;
	case B00000011000011: error = 0; break;
	case B00000011000001: error = 0; break;

	case B00000011100110: error = -1; break;
	case B00000011100111: error = -1; break;
	case B00000011100011: error = -1; break;
	case B00000011100001: error = -1; break;

	case B00000001100110: error = -2; break;
	case B00000001100111: error = -2; break;
	case B00000001100011: error = -2; break;
	case B00000001100001: error = -2; break;

	case B00000001110001: error = -3; break;

	case B00000000110001: error = -4; break;

	default: error = last_error;
	}

	float div_error = error - last_error;
	if(last_mv < 90 && last_mv > -90)
		if(!(sum_error < -32000 && error < 0) && !(sum_error > 32000 && error > 0) ) sum_error += error;
	temp_yaw_speed = last_mv;
	float p, i, d;
	if(flag == 1) {
//		p = _speed * PID_KKP * (float)error;
//		i = 0;
//		d = _speed * PID_KKD * (float)div_error / period;
//		p = (0.09*_speed + 0.8833) * (float)error;
//		i = (0.03*_speed + 0.6833) * (float)sum_error * period;
//		d = (0.7433*exp(0.1099*_speed)) * (float)div_error / period;
		p = _speed * 0.075 * (float)error;
		i = 0;
		d = 0;
	}
	else if(flag == 2) {
		p = kp * (float)error;
		i = 0;
		d = 0;
	}
	else {
		p = kp * (float)error;
		i = ki * (float)sum_error * period;
		d = kd * (float)div_error / period;
	}
	d = alpha*d + (1.00-alpha)*last_d;
	last_d = d;
	float mv = p + i + d;
	last_error = error;
	last_mv = mv;
	return mv;
}

void Run_SetKP(float kp) {

}


void Run_LineTracing(float speed, float period, uint8_t flag) {

//	static float last_speed = 0;
//
//	float temp_accl = this_accl * period * 10.00;
//
//	if(speed > last_speed+temp_accl && temp_accl != 0) {
//		last_speed += temp_accl;
//	}
//	else {
//		last_speed = speed;
//	}
//
//	speed = last_speed;

	float yaw_speed = Run_YawSpeed(period, speed, flag);
	float temp_speed = speed;
	if(yaw_speed > 99) yaw_speed = 99;
	else if(yaw_speed < -99) yaw_speed = -99;
	if(speed+yaw_speed > 99) {
		temp_speed-=(speed+yaw_speed-99);
	}
	else if(speed-yaw_speed < -99) {
		temp_speed+=(speed-yaw_speed+99);
	}
	Motor_SetPoint(&MOTOR_R, temp_speed-yaw_speed);
	Motor_SetPoint(&MOTOR_L, temp_speed+yaw_speed);
}

uint8_t Run_MotorNotOver() {
	return (Motor_GetPoint(&MOTOR_R) < 99) && (Motor_GetPoint(&MOTOR_R) > -99)
		&& (Motor_GetPoint(&MOTOR_L) < 99) && (Motor_GetPoint(&MOTOR_L) > -99) ;
}

void Run_SetMotorSpeed(float speed_l, float speed_r) {
	Motor_SetPoint(&MOTOR_R, speed_r);
	Motor_SetPoint(&MOTOR_L, speed_l);
}

void Run_SetMotorAccl(float accl) {
//	Motor_SetAccl(&MOTOR_R, accl);
//	Motor_SetAccl(&MOTOR_L, accl);
	this_accl = accl;
}

void Run_SetReverseSpeed(float factor) {
	Motor_SetPoint(&MOTOR_R, -Motor_GetPoint(&MOTOR_R)*factor);
	Motor_SetPoint(&MOTOR_L, -Motor_GetPoint(&MOTOR_L)*factor);
}
