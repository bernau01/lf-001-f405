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

#define MOTOR_L motor[1]
#define MOTOR_R motor[0]

extern float kp, ki, kd;
extern Motor_typedef motor[2];

void Run_Init();
void Run_MotorRoutine(float period);
void Run_LineTracing(float speed, float period, uint8_t flag);
void Run_SetMotorSpeed(float speed_l, float speed_r);

#endif /* INC_RUN_H_ */
