/*
 * run.c
 *
 *  Created on: Jul 29, 2023
 *      Author: Zain Irsyad
 */

#include "run.h"
#include "memory.h"
#include "bindef.h"


Motor_typedef motor[2];

float robot_enc_pos = 0;
float robot_enc_yawpos = 0;

void Run_Init() {
	memset(motor, 0, sizeof(motor[0])*2);

	motor[0].mode = MOTOR_MODE_OPEN;
	motor[0].pwm_htim = &htim3;
	motor[0].ch1 = TIM_CHANNEL_1;
	motor[0].ch2 = TIM_CHANNEL_2;
	motor[0].enc_htim = &htim2;
	motor[0].dir = 1;
	motor[0].pwm_factor = 0.009975;
	motor[0].vel_factor = 0.625;
	motor[0].kp = 2;
	motor[0].ki = 15;
	motor[0].filter_alpha = 0.5;

	motor[1].mode = motor[0].mode;
	motor[1].pwm_htim = &htim3;
	motor[1].ch1 = TIM_CHANNEL_4;
	motor[1].ch2 = TIM_CHANNEL_3;
	motor[1].enc_htim = &htim4;
	motor[1].dir = -1;
	motor[1].pwm_factor = 0.01;
	motor[1].vel_factor = 0.625;
	motor[1].kp = motor[0].kp;
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


float kp;
float ki;
float kd;
uint16_t sum_error;

float Run_YawSpeed(float period, float _speed, uint8_t flag) {
	static int16_t last_error = 0;
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
	case B00000000000001: error = -15; break;
	case B00000000000011: error = -13; break;
	case B00000000000111: error = -11; break;
	case B00000000001111: error = -10; break;
	case B00000000011111: error = -9; break;
	case B00000000011110: error = -8; break;
	case B00000000111110: error = -7; break;
	case B00000000111100: error = -6; break;
	case B00000001111100: error = -5; break;
	case B00000001111000: error = -4; break;
	case B00000011111000: error = -3; break;
	case B00000011110000: error = -2; break;
	case B00000111110000: error = -1; break;

	case B00000111100000: error = 0; break;

	case B00001111100000: error = 1; break;
	case B00001111000000: error = 2; break;
	case B00011111000000: error = 3; break;
	case B00011110000000: error = 4; break;
	case B00111110000000: error = 5; break;
	case B00111100000000: error = 6; break;
	case B01111100000000: error = 7; break;
	case B01111000000000: error = 8; break;
	case B11111000000000: error = 9; break;
	case B11110000000000: error = 10; break;
	case B11100000000000: error = 11; break;
	case B11000000000000: error = 13; break;
	case B10000000000000: error = 15; break;

	case B00000111111000: error = -1; break;
	case B00001111111100: error = -1; break;
	case B00011111111110: error = -1; break;

	case B00001111110000: error = 0; break;
	case B00011111111000: error = 0; break;
	case B00111111111100: error = 0; break;

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

	case B11000011110000: error = -2; break;
	case B10000111110000: error = -1; break;
	case B10000111100001: error = 0; break;
	case B00001111100001: error = 1; break;
	case B10001111000011: error = 2; break;

	case B11100011110000: error = -2; break;
//	case B11000011110000: error = -2; break;
	case B10000011110000: error = -2; break;

	case B11100111110000: error = -1; break;
	case B11000111110000: error = -1; break;
//	case B10000111110000: error = -1; break;

	case B11100111100000: error = 0; break;
	case B11000111100000: error = 0; break;
	case B10000111100000: error = 0; break;

	case B11101111100000: error = 1; break;
	case B11001111100000: error = 1; break;
	case B10001111100000: error = 1; break;

	case B11101111000000: error = 2; break;
	case B11001111000000: error = 2; break;
	case B10001111000000: error = 2; break;

	case B10011111000000: error = 3; break;

	case B10011110000000: error = 4; break;



	case B00001111000111: error = 2; break;
	case B00001111000011: error = 2; break;
	case B00001111000001: error = 2; break;

	case B00001111100111: error = 1; break;
	case B00001111100011: error = 1; break;
//	case B00001111100001: error = 1; break;

	case B00000111100111: error = 0; break;
	case B00000111100011: error = 0; break;
	case B00000111100001: error = 0; break;

	case B00000111110111: error = -1; break;
	case B00000111110011: error = -1; break;
	case B00000111110001: error = -1; break;

	case B00000011110111: error = -2; break;
	case B00000011110011: error = -2; break;
	case B00000011110001: error = -2; break;

	case B00000011111001: error = -3; break;

	case B00000001111001: error = -4; break;

	default: error = last_error;
	}

	int8_t div_error = error - last_error;
	sum_error += error;
	float p, i, d;
	if(flag == 1) {
		p = _speed * PID_KKP * (float)error;
		i = 0;
		d = _speed * PID_KKD * (float)div_error / period;
	}
	else {
		p = kp * (float)error;
		i = ki * (float)sum_error * period;
		d = kd * (float)div_error / period;
	}
	float mv = p + i + d;
	last_error = error;
	return mv;
}

void Run_LineTracing(float speed, float period, uint8_t flag) {
	float yaw_speed = Run_YawSpeed(period, speed, flag);
	Motor_SetPoint(&MOTOR_R, speed-yaw_speed);
	Motor_SetPoint(&MOTOR_L, speed+yaw_speed);
}

void Run_SetMotorSpeed(float speed_l, float speed_r) {
	Motor_SetPoint(&MOTOR_R, speed_r);
	Motor_SetPoint(&MOTOR_L, speed_l);
}
