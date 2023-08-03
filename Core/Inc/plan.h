/*
 * plan.h
 *
 *  Created on: Jul 31, 2023
 *      Author: Zain Irsyad
 */

#include "main.h"

#ifndef INC_PLAN_H_
#define INC_PLAN_H_

#define DELAY_FACTOR	10

#define PLAN_STOP		0
#define PLAN_LEFT		1
#define PLAN_RIGHT		2
#define PLAN_FORWARD	3
#define PLAN_FOLLOW		4
#define PLAN_FOLLOW_L	5
#define PLAN_FOLLOW_R	6
#define PLAN_INVERT		7
#define PLAN_NEXT		8
#define PLAN_NULL		9
#define PLAN_BACKWARD	10
#define PLAN_SKIP		11

void Plan_Start();
void Plan_UIRoutine();
void Plan_Main(float period);
uint8_t Plan_Stop();

#endif /* INC_PLAN_H_ */
