/*
 * run.h
 *
 *  Created on: Jul 29, 2023
 *      Author: Zain Irsyad
 */

#ifndef INC_RUN_H_
#define INC_RUN_H_

#include "main.h"
#include "motor.h"
#include "tim.h"
#include "sensor.h"

//#define PID_KKP	0.2042
//#define PID_KKD 1.8

#define MOTOR_L motor[1]
#define MOTOR_R motor[0]

extern float kp, ki, kd;
extern float alpha;
extern Motor_typedef motor[2];
extern int32_t robot_enc_pos;
extern float robot_enc_yawpos;
extern int16_t sum_error;
void Run_Init();
void Run_SetMotorAccl(float accl);
void Run_MotorRoutine(float period);
void Run_LineTracing(float speed, float period, uint8_t flag);
void Run_SetMotorSpeed(float speed_l, float speed_r);
void Run_SetReverseSpeed(float factor);
uint8_t Run_MotorNotOver();

#endif /* INC_RUN_H_ */
